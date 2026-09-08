#!/usr/bin/env python3
"""Friction vs angle map on the current build.

Slow constant-velocity position sweep of the output within a bounded arc,
logging torque continuously. With the 100 g arm (gravity torque 0.18 N·m max)
the torque log is essentially pure gearbox friction + ripple:
  - friction vs OUTPUT angle (both directions)  -> new friction_map.json data
  - the 2x/rotor-rev wave generator ripple (rotor angle = 50 x output angle)

    python3 characterize/friction_sweep.py             # +-80 deg from start, 0.05 rev/s, 2 cycles
    python3 characterize/friction_sweep.py 60 0.03 3   # arc_deg  vel_rps  cycles

START WITH THE ARM VERTICAL. The arc is centered on wherever it starts.
Uses the stored position gains (kp 8000 etc.), torque capped at MAX_TQ.
"""
import asyncio, csv, datetime, math, sys, time

ARC_DEG  = float(sys.argv[1]) if len(sys.argv) > 1 else 80.0
VEL_RPS  = float(sys.argv[2]) if len(sys.argv) > 2 else 0.05
CYCLES   = int(sys.argv[3])   if len(sys.argv) > 3 else 2
# asymmetric arc: pass LO HI (deg relative to start) as args 4 and 5;
# they override the symmetric +-ARC_DEG.
LO_DEG   = float(sys.argv[4]) if len(sys.argv) > 4 else -ARC_DEG
HI_DEG   = float(sys.argv[5]) if len(sys.argv) > 5 else ARC_DEG

MAX_TQ     = 18.0          # loaded sweeps need breakaway + gravity headroom; under 21 rated
ABORT_FET  = 70.0
PERIOD     = 0.005
WATCHDOG_S = 0.5
SETTLE_S   = 1.0                # pause at each end point

LIMITS = {96,97,98,99,100,101,102,103}

async def main():
    import moteus
    qr = moteus.QueryResolution()
    qr.mode=moteus.INT8; qr.fault=moteus.INT8; qr.position=moteus.F32
    qr.velocity=moteus.F32; qr.torque=moteus.F32
    qr.q_current=moteus.F32; qr.d_current=moteus.F32
    qr.voltage=moteus.F32; qr.temperature=moteus.INT8
    c = moteus.Controller(id=1, query_resolution=qr); R = moteus.Register

    await c.set_stop(); await asyncio.sleep(0.3)
    v0 = (await c.query()).values
    if int(v0[R.FAULT]) and int(v0[R.FAULT]) not in LIMITS:
        print(f"pre-existing fault {int(v0[R.FAULT])}, clear it first"); return 1
    home = v0[R.POSITION]
    lo, hi = LO_DEG/360.0, HI_DEG/360.0
    span = hi - lo
    print(f"arc {LO_DEG:+.0f}..{HI_DEG:+.0f} deg from start, {VEL_RPS} rev/s, "
          f"{CYCLES} cycles, torque cap {MAX_TQ} N·m")
    print(f"one half-sweep takes {span/VEL_RPS:.0f} s\n")

    # end points: hi then lo, repeated; return to start at the end.
    targets = []
    for _ in range(CYCLES):
        targets += [home + hi, home + lo]
    targets.append(home)

    rows, aborted = [], None
    t0 = time.monotonic()
    try:
        for ti, tgt in enumerate(targets):
            print(f"  leg {ti+1}/{len(targets)}: to {(tgt-home)*360:+.0f} deg", flush=True)
            k = 0; t_leg = time.monotonic()
            while True:
                r = await c.set_position(
                        position=tgt, velocity=0.0,
                        velocity_limit=VEL_RPS, accel_limit=VEL_RPS/0.5,
                        maximum_torque=MAX_TQ,
                        watchdog_timeout=WATCHDOG_S, query=True)
                v = r.values
                dpos = (v[R.POSITION] - home) * 360.0
                rows.append({"t_s": round(time.monotonic()-t0,3), "leg": ti+1,
                             "tgt_deg": round((tgt-home)*360,2),
                             "pos_deg": round(dpos,4),
                             "rotor_phase": round((v[R.POSITION]*50.0) % 1.0, 5),
                             "vel_rps": v[R.VELOCITY],
                             "torque_Nm": v[R.TORQUE], "iq_A": v[R.Q_CURRENT],
                             "bus_V": v[R.VOLTAGE], "fet_C": v[R.TEMPERATURE],
                             "fault": int(v[R.FAULT])})
                f = int(v[R.FAULT])
                if f and f not in LIMITS:
                    raise RuntimeError(f"fault {f}")
                if int(v[R.MODE]) == 1:
                    raise RuntimeError("controller faulted")
                if int(v[R.MODE]) == 11:
                    raise RuntimeError("watchdog timeout mode, command gap")
                if not (LO_DEG - 8.0 < dpos < HI_DEG + 8.0):
                    raise RuntimeError(f"position guard: {dpos:+.1f} deg")
                if v[R.TEMPERATURE] > ABORT_FET:
                    raise RuntimeError(f"fet {v[R.TEMPERATURE]:.0f} C")
                # leg done when close to target and essentially stopped
                if abs(v[R.POSITION]-tgt)*360 < 1.0 and abs(v[R.VELOCITY]) < 0.005:
                    break
                if time.monotonic()-t_leg > 2*(span/VEL_RPS) + 20:
                    raise RuntimeError("leg timeout, not reaching target")
                k += 1
                s = t_leg + k*PERIOD - time.monotonic()
                if s > 0: await asyncio.sleep(s)
            # settle at the end point WHILE feeding the watchdog (a plain sleep
            # longer than 0.5 s trips it and later commands are ignored)
            t_settle = time.monotonic()
            while time.monotonic() - t_settle < SETTLE_S:
                await c.set_position(position=tgt, velocity=0.0,
                                     velocity_limit=VEL_RPS, accel_limit=VEL_RPS/0.5,
                                     maximum_torque=MAX_TQ,
                                     watchdog_timeout=WATCHDOG_S, query=False)
                await asyncio.sleep(0.1)
    except (Exception, KeyboardInterrupt) as e:
        aborted = str(e) or type(e).__name__
        print(f"\nSTOP: {aborted}")
    finally:
        try: await c.set_stop()
        except Exception: pass
        print("stopped. motor is off.")

    if rows:
        import os; os.makedirs("data", exist_ok=True)
        stamp = f"{datetime.datetime.now():%Y%m%d_%H%M%S}"
        fn = f"data/friction_sweep_{stamp}.csv"
        with open(fn,"w",newline="") as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
        print("->", fn, f"({len(rows)} samples)")
    return 0

sys.exit(asyncio.run(main()))
