#!/usr/bin/env python3
"""No-load speed sweep, both directions, with a plot at the end.

Steps the output through eight speeds from 0.05 to 0.40 rev/s, dwells at each,
and logs everything. Run it forward, then backward, then compare: this is the
test that tells you what the gearbox costs you before any load is attached.

    python3 characterize/speed_sweep.py --dry-run     # connect, read, no motion
    python3 characterize/speed_sweep.py               # forward then reverse
    python3 characterize/speed_sweep.py --plot FILE   # replot an old CSV

--dry-run first, every time. It checks the query fits in one CAN-FD frame,
which is the failure that silently drops the FAULT register and blinds the
abort check.
"""
import argparse
import asyncio
import csv
import datetime
import math
import os
import sys
import time

from actuator.link import (make_query_resolution, verify_query, FAST_EXPECTED,
                           read_slow, AuxReader)
from actuator.registers import (unpack, MOTEUS_MODES, MOTEUS_FAULTS,
                                MOTEUS_LIMIT_REASONS)
from actuator.safety import (Abort, check_limits, ABORT_FET_TEMP_C,
                             ABORT_Q_CURRENT_A, ABORT_BUS_V_LOW, ABORT_BUS_V_HIGH)
from actuator.csvlog import load_csv
from actuator.plotting import (C_CMD, C_ACTUAL, C_Q, C_BUS, C_TORQUE, C_FET,
                               C_MOTOR, C_CRC)

CAN_ID = 1

SPEEDS_RPS_OUT = [0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40]

MAX_TORQUE_NM_OUT = 10.0   # output-referred. 10 N·m is well under the 44 N·m peak.

RAMP_S = 0.75      # smooth ramp between setpoints, avoids start/stop transients
DWELL_S = 4.0      # hold time at each speed
SETTLE_S = 1.5     # leading part of dwell discarded from steady-state stats
REST_S = 1.5       # coast at zero between direction blocks
SAMPLE_HZ = 50.0
AUX_POLL_HZ = 5.0  # diagnostic-channel poll rate (crc_errors, raw encoder counts)

CSV_FIELDS = [
    "t_s", "block", "phase", "cmd_velocity_rps_out",
    "mode", "position_rev_out", "velocity_rps_out", "torque_Nm_out",
    "q_current_A", "d_current_A",
    "bus_voltage_V", "power_W", "bus_current_A_derived",
    "fet_temp_C", "motor_temp_C", "fault",
    # fast encoder registers
    "orbis_position_rev_out", "orbis_velocity_rps_out", "encoder_validity",
    # slow diagnostic channel (forward-filled between polls)
    "bissc_raw", "bissc_crc_errors", "bissc_crc_delta",
    "as5047p_raw_ignore", "aux_age_s",
]



async def dry_run():
    import moteus

    qr = make_query_resolution(moteus)
    c = moteus.Controller(id=CAN_ID, query_resolution=qr)

    print(f"Querying CAN id {CAN_ID}, commanding NO motion ...\n")
    await c.set_stop()
    await asyncio.sleep(0.2)

    for i in range(5):
        result = await c.query()
        slow = await read_slow(moteus, c)
        s = unpack(moteus, result, slow)
        if i == 4:
            missing = verify_query(moteus, result)
            if missing:
                print(f"  *** QUERY TRUNCATED: missing {missing}")
                print("  *** Reply exceeded one CAN-FD frame. DO NOT RUN THE SWEEP.")
            else:
                print(f"  query complete   all {len(FAST_EXPECTED)} fast registers present")
            print(f"  mode              {s['mode']} ({MOTEUS_MODES.get(s['mode'], '?')})")
            print(f"  fault             {s['fault']} "
                  f"({MOTEUS_FAULTS.get(s['fault'], 'none' if s['fault'] == 0 else '?')})")
            print(f"  position          {s['position_rev_out']:+.4f} rev (output)")
            print(f"  velocity          {s['velocity_rps_out']:+.4f} rev/s (output)")
            print(f"  bus voltage       {s['bus_voltage_V']:.2f} V")
            print(f"  power             {s['power_W']:.2f} W")
            print(f"  fet_temp_C        {s['fet_temp_C']:.1f} C")
            print(f"  motor_temp_C      {s['motor_temp_C']:.1f} C  "
                  f"(0 expected, no thermistor, not used for aborts)")
            print(f"  encoder_validity  {s['encoder_validity']:.0f}  (slow poll)")
            print(f"  ENCODER_0 (Orbis) {s['orbis_position_rev_out']:+.4f} rev, "
                  f"{s['orbis_velocity_rps_out']:+.4f} rev/s")
        await asyncio.sleep(0.1)

    print("\nDiagnostic channel (aux1) ...")
    aux = AuxReader(moteus, c)
    ok = await aux.connect()
    if not ok:
        print(f"  UNAVAILABLE: {aux.error}")
        print("  The sweep will still run; crc columns will be nan.")
    else:
        print(f"  discovered fields:")
        print(f"    crc_errors  -> {aux.key_crc}")
        print(f"    bissc value -> {aux.key_bissc}")
        print(f"    spi value   -> {aux.key_spi}  (AS5047P, meaningless, logged only)")
        print(f"  CRC BASELINE: {aux.baseline_crc:.0f}"
              f"   <- deltas during the sweep are measured from this")
        if aux.baseline_crc and abs(aux.baseline_crc - 694) > 0.5:
            print("  note: baseline is not the 694 I recorded at calibration")

        print("\n  Watching crc for 3 s at rest (motor not switching) ...")
        t_end = time.monotonic() + 3.0
        seen = []
        while time.monotonic() < t_end:
            _, crc, delta, _ = await aux.sample()
            seen.append(delta)
            await asyncio.sleep(1.0 / AUX_POLL_HZ)
        rise = max(seen) - min(seen) if seen else float("nan")
        print(f"  delta over {len(seen)} polls: {min(seen):.0f} -> {max(seen):.0f} "
              f"({'STATIC: good' if rise == 0 else f'CLIMBING by {rise:.0f} AT REST'})")

        print("\n  Full aux1 telemetry dump:")
        for k, v in sorted(aux.last_flat.items()):
            print(f"    {k:<34} {v}")

    await c.set_stop()
    print("\nDry run OK. Controller left stopped. Nothing was commanded to move.")



async def run_sweep(directions, csv_path):
    import moteus

    qr = make_query_resolution(moteus)
    c = moteus.Controller(id=CAN_ID, query_resolution=qr)

    rows = []
    period = 1.0 / SAMPLE_HZ
    aux_period = 1.0 / AUX_POLL_HZ
    t0 = None
    aborted = None

    aux = AuxReader(moteus, c)
    # Latest diagnostic sample, forward-filled into every row until refreshed.
    nan = float("nan")
    aux_state = {"bissc": nan, "crc": nan, "delta": nan, "spi": nan, "t": None}
    slow_state = {}
    next_aux_t = [0.0]

    def log(block, phase, cmd_v, s, t):
        age = (t - aux_state["t"]) if aux_state["t"] is not None else nan
        rows.append({
            "t_s": round(t, 4),
            "block": block,
            "phase": phase,
            "cmd_velocity_rps_out": round(cmd_v, 5),
            **s,
            "bissc_raw": aux_state["bissc"],
            "bissc_crc_errors": aux_state["crc"],
            "bissc_crc_delta": aux_state["delta"],
            "as5047p_raw_ignore": aux_state["spi"],
            "aux_age_s": round(age, 4) if not math.isnan(age) else nan,
        })

    async def hold(block, phase, cmd_fn, duration):
        """Command a (possibly time-varying) velocity for `duration`, logging."""
        nonlocal t0
        start = time.monotonic()
        n = 0
        while True:
            now = time.monotonic()
            elapsed = now - start
            if elapsed >= duration:
                return
            cmd_v = cmd_fn(elapsed)

            result = await c.set_position(
                position=math.nan,
                velocity=cmd_v,
                maximum_torque=MAX_TORQUE_NM_OUT,
                query=True,
            )
            s = unpack(moteus, result, slow_state)
            if t0 is None:
                t0 = now
            t_rel = now - t0

            # Decimated slow poll, sequential with the control cycle so the
            # two never contend for the link.
            if t_rel >= next_aux_t[0]:
                slow_state.update(await read_slow(moteus, c))
                if aux.available:
                    b, crc, delta, spi = await aux.sample()
                    aux_state.update(bissc=b, crc=crc, delta=delta,
                                     spi=spi, t=t_rel)
                next_aux_t[0] = t_rel + aux_period
                s = unpack(moteus, result, slow_state)

            log(block, phase, cmd_v, s, t_rel)
            check_limits(s)

            n += 1
            sleep_for = (start + n * period) - time.monotonic()
            if sleep_for > 0:
                await asyncio.sleep(sleep_for)

    try:
        print("Clearing faults / stopping ...")
        await c.set_stop()
        await asyncio.sleep(0.3)

        pre_result = await c.query()
        missing = verify_query(moteus, pre_result)
        if missing:
            raise Abort(f"query reply truncated, missing {missing}, "
                        f"telemetry would be silently incomplete")

        slow_state.update(await read_slow(moteus, c))
        pre = unpack(moteus, pre_result, slow_state)
        print(f"Bus {pre['bus_voltage_V']:.2f} V, fet {pre['fet_temp_C']:.1f} C, "
              f"fault {pre['fault']}, encoder_validity "
              f"{pre['encoder_validity']:.0f}\n")
        if pre["fault"]:
            raise Abort(f"pre-existing fault {pre['fault']} "
                        f"({MOTEUS_FAULTS.get(pre['fault'], '?')}), diagnose first")

        if await aux.connect():
            print(f"aux1 diagnostic OK. CRC baseline {aux.baseline_crc:.0f} "
                  f"(field '{aux.key_crc}'), polling at {AUX_POLL_HZ:g} Hz\n")
        else:
            print(f"aux1 diagnostic UNAVAILABLE: {aux.error}")
            print("Proceeding without CRC logging; those columns will be nan.\n")

        for sign, label in directions:
            print(f"--- {label} ---")
            prev = 0.0
            for target in SPEEDS_RPS_OUT:
                v_target = sign * target

                # ramp
                v_from, v_to = prev, v_target
                await hold(label, "ramp",
                           lambda e, a=v_from, b=v_to: a + (b - a) * min(e / RAMP_S, 1.0),
                           RAMP_S)

                # dwell
                await hold(label, "dwell", lambda e, b=v_to: b, DWELL_S)
                prev = v_target

                # quick console readback of the last sample
                last = rows[-1]
                crc_d = last["bissc_crc_delta"]
                crc_s = "crc +0" if crc_d == 0 else (
                    f"crc +{crc_d:.0f} <-- CLIMBING" if not math.isnan(crc_d)
                    else "crc n/a")
                lim = MOTEUS_LIMIT_REASONS.get(last["fault"])
                lim_s = f", LIMITED BY {lim}" if lim else ""
                print(f"  {v_target:+.2f} rev/s -> actual {last['velocity_rps_out']:+.3f}, "
                      f"Iq {last['q_current_A']:+.2f} A, "
                      f"Ibus {last['bus_current_A_derived']:+.2f} A, "
                      f"fet {last['fet_temp_C']:.1f} C, {crc_s}{lim_s}")

            # ramp back to zero and coast
            v_from = prev
            await hold(label, "ramp",
                       lambda e, a=v_from: a * (1.0 - min(e / RAMP_S, 1.0)),
                       RAMP_S)
            await hold(label, "rest", lambda e: 0.0, REST_S)
            print()

    except Abort as e:
        aborted = str(e)
        print(f"\n*** ABORT: {e}", file=sys.stderr)
    except KeyboardInterrupt:
        aborted = "keyboard interrupt"
        print("\n*** Interrupted by user", file=sys.stderr)
    finally:
        try:
            await c.set_stop()
            print("Motor stopped.")
        except Exception as e:
            print(f"!! set_stop failed: {e}, CUT POWER", file=sys.stderr)

    if rows:
        with open(csv_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
            w.writeheader()
            w.writerows(rows)
        print(f"Wrote {len(rows)} samples -> {csv_path}")

    return rows, aborted



def steady_state_table(rows):
    """Mean of each dwell window, excluding the settling lead-in."""
    groups = {}
    for r in rows:
        if r["phase"] != "dwell":
            continue
        key = (r["block"], round(r["cmd_velocity_rps_out"], 4))
        groups.setdefault(key, []).append(r)

    table = []
    for (block, cmd), pts in groups.items():
        if not pts:
            continue
        t_start = pts[0]["t_s"]
        keep = [p for p in pts if p["t_s"] - t_start >= SETTLE_S] or pts

        def mean(k):
            vals = [p[k] for p in keep if not math.isnan(p[k])]
            return sum(vals) / len(vals) if vals else float("nan")

        table.append({
            "block": block,
            "cmd": cmd,
            "n": len(keep),
            "velocity": mean("velocity_rps_out"),
            "iq": mean("q_current_A"),
            "ibus": mean("bus_current_A_derived"),
            "torque": mean("torque_Nm_out"),
            "power": mean("power_W"),
            "fet": mean("fet_temp_C"),
            "vbus": mean("bus_voltage_V"),
            "crc_delta_end": next(
                (p["bissc_crc_delta"] for p in reversed(keep)
                 if not math.isnan(p["bissc_crc_delta"])), float("nan")),
        })
    table.sort(key=lambda r: (r["block"], abs(r["cmd"])))
    return table



def crc_report(rows):
    """Did BiSS-C CRC errors accumulate once the phase wires started switching?"""
    vals = [(r["t_s"], r["bissc_crc_delta"]) for r in rows
            if not math.isnan(r["bissc_crc_delta"])]
    print("\n" + "=" * 74)
    print("BiSS-C LINK INTEGRITY  (unshielded cable, no drain ground)")
    print("=" * 74)
    if not vals:
        print("  No CRC data captured, diagnostic channel was unavailable.")
        return None

    first, last = vals[0][1], vals[-1][1]
    total = last - first
    moving = [d for (t, d), r in zip(vals, [r for r in rows
              if not math.isnan(r["bissc_crc_delta"])])
              if abs(r["cmd_velocity_rps_out"]) > 1e-6]
    growth_moving = (max(moving) - min(moving)) if moving else float("nan")

    print(f"  new errors over the whole run   {total:+.0f}")
    print(f"  growth while commanding motion  {growth_moving:+.0f}")
    if total == 0:
        print("\n  VERDICT: CLEAN. Zero new CRC errors with the phases switching.")
        print("  The unshielded run survived this test at these currents.")
    else:
        rate = total / vals[-1][0] if vals[-1][0] else float("nan")
        print(f"  average rate                    {rate:.2f} errors/s")
        print("\n  VERDICT: *** CRC ERRORS ACCUMULATED ***")
        print("  The encoder link is picking up switching noise. Commutation")
        print("  runs off this encoder, so this must be fixed before the")
        print("  gearbox goes on. Do not load the flexspline against a")
        print("  commutation source that is dropping frames.")
        print("  The crc+ column in the table above shows where it started.")
    return total



def print_table(table):
    if not table:
        return
    print("\nSteady-state summary (dwell means, first "
          f"{SETTLE_S:g}s of each step discarded)")
    print(f"{'block':<9} {'cmd':>7} {'actual':>8} {'track%':>7} {'Iq':>7} "
          f"{'Ibus':>7} {'torque':>8} {'P':>7} {'Vbus':>6} {'fet':>6} {'crc+':>6}")
    print(f"{'':<9} {'rev/s':>7} {'rev/s':>8} {'':>7} {'A':>7} "
          f"{'A':>7} {'N·m':>8} {'W':>7} {'V':>6} {'C':>6} {'cum':>6}")
    print("-" * 90)
    for r in table:
        track = 100.0 * r["velocity"] / r["cmd"] if r["cmd"] else float("nan")
        cd = r["crc_delta_end"]
        cds = "  n/a" if math.isnan(cd) else f"{cd:+.0f}"
        print(f"{r['block']:<9} {r['cmd']:>+7.2f} {r['velocity']:>+8.3f} "
              f"{track:>6.1f}% {r['iq']:>+7.2f} {r['ibus']:>+7.2f} "
              f"{r['torque']:>+8.3f} {r['power']:>7.2f} {r['vbus']:>6.2f} "
              f"{r['fet']:>6.1f} {cds:>6}")



def plot(rows, csv_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = [r["t_s"] for r in rows]

    def col(k):
        return [r[k] for r in rows]

    def style(ax, ylabel):
        ax.set_ylabel(ylabel)
        ax.grid(True, color="#DDDDDD", linewidth=0.6)
        ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            ax.spines[side].set_color("#AAAAAA")
        ax.tick_params(colors="#444444", labelsize=9)

    # ---- Figure 1: time series, one measure per panel (never a dual axis) ----
    fig, axes = plt.subplots(5, 1, figsize=(11, 13), sharex=True)

    axes[0].plot(t, col("cmd_velocity_rps_out"), color=C_CMD, lw=2,
                 ls="--", label="commanded")
    axes[0].plot(t, col("velocity_rps_out"), color=C_ACTUAL, lw=2, label="actual")
    axes[0].axhline(0.46, color=C_CMD, lw=1, ls=":")
    axes[0].axhline(-0.46, color=C_CMD, lw=1, ls=":")
    axes[0].text(t[-1] if t else 0, 0.46, " motor_max_velocity", fontsize=8,
                 color="#666666", va="bottom", ha="right")
    style(axes[0], "velocity\n(output rev/s)")
    axes[0].legend(frameon=False, fontsize=9, loc="upper left")

    axes[1].plot(t, col("q_current_A"), color=C_Q, lw=2, label="q-axis current")
    axes[1].plot(t, col("bus_current_A_derived"), color=C_BUS, lw=2,
                 label="bus current (derived P/V)")
    axes[1].axhline(7.5, color="#B00020", lw=1, ls=":")
    axes[1].axhline(-7.5, color="#B00020", lw=1, ls=":")
    style(axes[1], "current (A)")
    axes[1].legend(frameon=False, fontsize=9, loc="upper left")

    axes[2].plot(t, col("torque_Nm_out"), color=C_TORQUE, lw=2)
    style(axes[2], "torque\n(N·m, output)")
    axes[2].set_title("Output torque", fontsize=10, color="#444444", loc="left")

    axes[3].plot(t, col("fet_temp_C"), color=C_FET, lw=2, label="FET temp")
    axes[3].plot(t, col("motor_temp_C"), color=C_MOTOR, lw=2,
                 label="motor temp (no thermistor, always 0)")
    style(axes[3], "temperature (°C)")
    axes[3].legend(frameon=False, fontsize=9, loc="upper left")

    # CRC panel: the headline result of this run.
    crc = col("bissc_crc_delta")
    axes[4].plot(t, crc, color=C_CRC, lw=2)
    style(axes[4], "new BiSS-C\nCRC errors")
    finite = [v for v in crc if not math.isnan(v)]
    if finite and max(finite) == 0:
        axes[4].set_ylim(-1, 1)
        axes[4].text(0.5, 0.5, "zero new CRC errors, link clean under switching",
                     transform=axes[4].transAxes, ha="center", va="center",
                     fontsize=11, color="#009E73")
    axes[4].set_title("BiSS-C errors accrued since run start "
                      "(baseline subtracted)", fontsize=10,
                      color="#444444", loc="left")
    axes[4].set_xlabel("time (s)")

    fig.suptitle(f"Motor-only velocity sweep, no gearbox, "
                 f"torque limit {MAX_TORQUE_NM_OUT:g} N·m output",
                 fontsize=12, y=0.995)
    fig.tight_layout()
    p1 = csv_path.replace(".csv", "_timeseries.png")
    fig.savefig(p1, dpi=150)
    plt.close(fig)

    # ---- Figure 2: steady-state vs commanded speed ----
    table = steady_state_table(rows)
    if table:
        fig2, ax2 = plt.subplots(1, 3, figsize=(13, 4))
        blocks = sorted({r["block"] for r in table})
        markers = {b: m for b, m in zip(blocks, ["o", "s"])}

        for b in blocks:
            sub = [r for r in table if r["block"] == b]
            x = [abs(r["cmd"]) for r in sub]
            ax2[0].plot(x, [abs(r["velocity"]) for r in sub], color=C_ACTUAL,
                        lw=2, marker=markers[b], ms=8, label=b)
            ax2[1].plot(x, [abs(r["iq"]) for r in sub], color=C_Q,
                        lw=2, marker=markers[b], ms=8, label=b)
            ax2[2].plot(x, [abs(r["ibus"]) for r in sub], color=C_BUS,
                        lw=2, marker=markers[b], ms=8, label=b)

        lim = max(abs(r["cmd"]) for r in table)
        ax2[0].plot([0, lim], [0, lim], color=C_CMD, lw=1.5, ls="--",
                    label="ideal 1:1")
        for a, lab in zip(ax2, ["|velocity| achieved (rev/s)",
                                "|q-axis current| (A)",
                                "|bus current| (A, derived)"]):
            style(a, lab)
            a.set_xlabel("commanded |velocity| (output rev/s)")
            a.legend(frameon=False, fontsize=9)

        fig2.suptitle("Steady-state per speed step (dwell means)",
                      fontsize=12, y=1.02)
        fig2.tight_layout()
        p2 = csv_path.replace(".csv", "_steadystate.png")
        fig2.savefig(p2, dpi=150, bbox_inches="tight")
        plt.close(fig2)
        print(f"Plots -> {p1}\n         {p2}")
    else:
        print(f"Plot  -> {p1}")

    print_table(table)
    crc_report(rows)



def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dry-run", action="store_true",
                    help="connect and read telemetry only, command no motion")
    ap.add_argument("--plot-only", metavar="CSV",
                    help="re-plot an existing CSV, no hardware needed")
    ap.add_argument("--forward-only", action="store_true",
                    help="skip the reverse-direction block")
    ap.add_argument("--outdir", default="data")
    args = ap.parse_args()

    if args.plot_only:
        rows = load_csv(args.plot_only)
        plot(rows, args.plot_only)
        return

    if args.dry_run:
        asyncio.run(dry_run())
        return

    directions = [(+1.0, "forward")]
    if not args.forward_only:
        directions.append((-1.0, "reverse"))

    n_steps = len(SPEEDS_RPS_OUT) * len(directions)
    est = n_steps * (RAMP_S + DWELL_S) + len(directions) * (RAMP_S + REST_S)

    os.makedirs(args.outdir, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(args.outdir, f"motor_sweep_{stamp}.csv")

    print("=" * 70)
    print("MOTOR-ONLY VELOCITY SWEEP  (gearbox must NOT be attached)")
    print("=" * 70)
    print(f"  speeds        {SPEEDS_RPS_OUT} output rev/s")
    print(f"  directions    {', '.join(d[1] for d in directions)}")
    print(f"  torque limit  {MAX_TORQUE_NM_OUT:g} N·m output")
    print(f"  abort on      fault | fet>{ABORT_FET_TEMP_C:g}C | "
          f"|Iq|>{ABORT_Q_CURRENT_A:g}A | Vbus outside "
          f"[{ABORT_BUS_V_LOW:g},{ABORT_BUS_V_HIGH:g}]")
    print(f"  duration      ~{est:.0f} s")
    print(f"  csv           {csv_path}")
    print()
    print("  Top speed 0.40 rev/s is ~87% of motor_max_velocity (0.46 at 24 V).")
    print("  Expect the top one or two steps to under-track. That is data, not a fault.")
    print()
    print("  Confirm: motor free to spin, nothing coupled to the shaft, hand on")
    print("  the PSU output switch.")
    print()

    try:
        resp = input("Type GO to start, anything else to abort: ").strip()
    except (EOFError, KeyboardInterrupt):
        resp = ""
    if resp != "GO":
        print("Aborted, nothing commanded.")
        return

    rows, aborted = asyncio.run(run_sweep(directions, csv_path))

    if rows:
        plot(rows, csv_path)
    if aborted:
        print(f"\nRun ended early: {aborted}")
        sys.exit(1)


if __name__ == "__main__":
    main()
