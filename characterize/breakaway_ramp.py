#!/usr/bin/env python3
"""
Auto-escalating breakaway torque finder.

Commands a fixed velocity target (so a stalled motor keeps demanding more
torque as long as it isn't moving) while stepping the torque CEILING itself
upward in increments. Watches the Orbis (unwrapped, output-referred) for
real movement. Stops the moment movement is detected, or the moment the
safety cap is reached with no movement, whichever comes first. No manual
step-by-step confirmation between torque steps; one GO starts the whole
ramp.

SAFETY_CAP_NM = 4.0
  - ~38% of rated torque (21 N·m)
  - ~24% of momentary max (91 N·m)
  - ~18% of the do-not-exceed peak (44 N·m)
  - current at 8 N·m / Kt~5.03 N·m/A ~= 1.6 A, well under servo.max_current_A
    (7.5A) and under the existing hard current abort (6.0A)
Change SAFETY_CAP_NM below if you want a different ceiling, the escalation
logic doesn't care what the number is, it just stops there.

The existing hard aborts (fault, current, temperature, voltage, CRC growth
report) still apply underneath this, the ramp operates INSIDE them, it
doesn't replace them.

RUN WITH:  /opt/anaconda3/bin/python3 characterize/breakaway_ramp.py
"""

import asyncio
import csv
import datetime
import math
import os
import sys
import time

import actuator as base

CAN_ID = 1

TARGET_VELOCITY_RPS_OUT = -0.20
                                  # error while stalled; not swept
TORQUE_STEP_NM = 1.0
TORQUE_START_NM = 3.0
                         # already tested clean, no movement, no point repeating
# 25 N·m output ~= 0.50 N·m at the motor ~= 5.0 A, chosen with real margin
# under BOTH constraints: 19 N·m below the 44 N·m do-not-exceed peak, and
# 1A below the 6.0A self-imposed current abort (2.5A below the 7.5A hard
# controller ceiling). This is close to the practical top of what's testable
# without eating into that margin, not a round number, a bounded one.
SAFETY_CAP_NM = 4.0
HOLD_PER_STEP_S = 2.5
SAMPLE_HZ = 100.0

# Movement detection: net Orbis travel over one step, converted to output
# revolutions. The Orbis is 14-bit absolute, clean at rest (near-zero drift
# in every prior dry-run check), this threshold is well above encoder
# noise and well below "it actually turned."
MOVE_THRESHOLD_OUTPUT_REV = 0.003   # ~1 degree output

CONFIRM_HOLD_S = 3.0
                        # it's sustained motion, not a single noise blip

CSV_FIELDS = ["t_s", "phase", "torque_step_Nm", "mode", "velocity_rps_out",
              "orbis_unwrapped_output_rev", "q_current_A", "bus_voltage_V",
              "fet_temp_C", "fault", "bissc_crc_delta"]


def unwrap_orbis(prev_unwrapped, prev_raw, raw):
    """ENCODER_0_POSITION wraps every ROTOR revolution (1/50 output rev).
    Unwrap it, then convert rotor revs to output revs (/50)."""
    if prev_raw is None:
        return prev_unwrapped, raw
    d = raw - prev_raw
    if d > 0.5:
        d -= 1.0
    elif d < -0.5:
        d += 1.0
    return prev_unwrapped + d, raw


async def main():
    import moteus

    c = moteus.Controller(id=CAN_ID,
                          query_resolution=base.make_query_resolution(moteus))
    aux = base.AuxReader(moteus, c)

    pre = await c.query()
    miss = base.verify_query(moteus, pre)
    if miss:
        print(f"*** query truncated, missing {miss}, aborting", file=sys.stderr)
        return 1
    p = base.unpack(moteus, pre)
    if p["fault"]:
        print(f"*** pre-existing fault {p['fault']}, diagnose first", file=sys.stderr)
        return 1

    n_steps = int((SAFETY_CAP_NM - TORQUE_START_NM) / TORQUE_STEP_NM) + 1
    est_s = n_steps * HOLD_PER_STEP_S

    print("=" * 72)
    print("AUTO-ESCALATING BREAKAWAY TORQUE FINDER")
    print("=" * 72)
    print(f"  target velocity     {TARGET_VELOCITY_RPS_OUT} output rev/s (fixed)")
    print(f"  torque steps        {TORQUE_START_NM:.1f} -> {SAFETY_CAP_NM:.1f} N·m, "
          f"+{TORQUE_STEP_NM:.1f} N·m every {HOLD_PER_STEP_S:.1f}s ({n_steps} steps)")
    print(f"  SAFETY CAP          {SAFETY_CAP_NM:.1f} N·m "
          f"(~{100*SAFETY_CAP_NM/21:.0f}% of 21 N·m rated, "
          f"~{100*SAFETY_CAP_NM/44:.0f}% of 44 N·m do-not-exceed peak)")
    print(f"  move threshold      {MOVE_THRESHOLD_OUTPUT_REV*360:.2f} deg output")
    print(f"  hard aborts         fault | fet>{base.ABORT_FET_TEMP_C:g}C | "
          f"|Iq|>{base.ABORT_Q_CURRENT_A:g}A | Vbus outside "
          f"[{base.ABORT_BUS_V_LOW:g},{base.ABORT_BUS_V_HIGH:g}]")
    print(f"  worst-case duration ~{est_s:.0f}s if breakaway is never found")
    print()
    print(f"  Bus {p['bus_voltage_V']:.2f} V, fet {p['fet_temp_C']:.1f} C, "
          f"fault {p['fault']}")
    print()
    print("  One GO runs the WHOLE ramp, no per-step confirmation. Watch the")
    print("  assembly continuously. Hand on the PSU switch. Cut power")
    print("  yourself if anything looks or sounds wrong, at any step.")
    print()

    try:
        resp = input("Type GO to start the ramp, anything else to abort: ").strip()
    except (EOFError, KeyboardInterrupt):
        resp = ""
    if resp != "GO":
        print("Aborted, nothing commanded.")
        return 0

    await aux.connect()
    if aux.available:
        print(f"aux1 diagnostic OK. CRC baseline {aux.baseline_crc:.0f}")
    else:
        print(f"aux1 diagnostic UNAVAILABLE: {aux.error} (continuing without CRC log)")

    rows = []
    period = 1.0 / SAMPLE_HZ
    t0 = None
    aborted = None
    found_at = None
    unwrapped = 0.0
    prev_raw = None
    aux_state = {"delta": float("nan")}
    next_aux_t = 0.0

    async def hold(phase, torque_nm, v_target, duration):
        nonlocal t0, unwrapped, prev_raw, next_aux_t
        start = time.monotonic()
        step_start_unwrapped = unwrapped
        k = 0
        while True:
            now = time.monotonic()
            t = now - start
            if t >= duration:
                break

            result = await c.set_position(
                position=math.nan, velocity=v_target,
                maximum_torque=torque_nm, query=True)
            s = base.unpack(moteus, result)
            if t0 is None:
                t0 = now
            t_rel = now - t0

            unwrapped, prev_raw = unwrap_orbis(
                unwrapped, prev_raw, s["orbis_position_rev_out"])

            if aux.available and t_rel >= next_aux_t:
                _, _, delta, _ = await aux.sample()
                aux_state["delta"] = delta
                next_aux_t = t_rel + 0.2  # 5 Hz

            rows.append({
                "t_s": round(t_rel, 4), "phase": phase,
                "torque_step_Nm": round(torque_nm, 2),
                "mode": s["mode"], "velocity_rps_out": s["velocity_rps_out"],
                "orbis_unwrapped_output_rev": round(unwrapped / 50.0, 6),
                "q_current_A": s["q_current_A"],
                "bus_voltage_V": s["bus_voltage_V"],
                "fet_temp_C": s["fet_temp_C"], "fault": s["fault"],
                "bissc_crc_delta": aux_state["delta"],
            })
            base.check_limits(s)

            k += 1
            slp = (start + k * period) - time.monotonic()
            if slp > 0:
                await asyncio.sleep(slp)

        traveled = (unwrapped - step_start_unwrapped) / 50.0
        return traveled

    try:
        torque = TORQUE_START_NM
        while torque <= SAFETY_CAP_NM + 1e-9:
            print(f"  step {torque:.1f} N·m ...", end=" ", flush=True)
            traveled = await hold("escalate", torque, TARGET_VELOCITY_RPS_OUT,
                                  HOLD_PER_STEP_S)
            last = rows[-1]
            crc_d = last["bissc_crc_delta"]
            crc_s = "crc +0" if crc_d == 0 else (
                f"crc +{crc_d:.0f}" if not math.isnan(crc_d) else "crc n/a")
            print(f"traveled {traveled*360:+.2f} deg output, "
                  f"Iq {last['q_current_A']:+.2f}A, fet {last['fet_temp_C']:.1f}C, "
                  f"{crc_s}")

            if abs(traveled) > MOVE_THRESHOLD_OUTPUT_REV:
                found_at = torque
                print(f"\n  *** MOVEMENT DETECTED at {torque:.1f} N·m, "
                      f"holding {CONFIRM_HOLD_S:.1f}s to confirm ...")
                confirm_traveled = await hold(
                    "confirm", torque, TARGET_VELOCITY_RPS_OUT, CONFIRM_HOLD_S)
                print(f"  confirm hold: traveled {confirm_traveled*360:+.2f} deg "
                      f"({'SUSTAINED: real breakaway' if abs(confirm_traveled) > MOVE_THRESHOLD_OUTPUT_REV else 'DID NOT SUSTAIN, was a blip, not true breakaway'})")
                break

            torque += TORQUE_STEP_NM

        else:
            pass

        if found_at is None and torque > SAFETY_CAP_NM:
            print(f"\n  Reached safety cap ({SAFETY_CAP_NM:.1f} N·m) with no "
                  f"movement detected. Breakaway torque exceeds the cap -")
            print(f"  stopping here rather than raising it automatically.")

    except base.Abort as e:
        aborted = str(e)
        print(f"\n*** ABORT: {e}", file=sys.stderr)
    except KeyboardInterrupt:
        aborted = "keyboard interrupt"
    finally:
        try:
            await c.set_stop()
            print("\nMotor commanded to stop.")
        except Exception as e:
            print(f"!! set_stop failed: {e}, CUT POWER NOW", file=sys.stderr)

    if rows:
        os.makedirs("data", exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = f"data/breakaway_ramp_{stamp}.csv"
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
            w.writeheader()
            w.writerows(rows)
        print(f"Wrote {len(rows)} samples -> {path}")

    if aborted:
        print(f"\nRun ended early: {aborted}")
        return 1
    if found_at is not None:
        print(f"\nBreakaway torque found: ~{found_at:.1f} N·m output.")
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
