#!/usr/bin/env python3
"""Measure REAL output torque against a lever arm and scale.

The only measurement in this project that does not take moteus's word for
anything. moteus reports Iq x Kt x 50, the torque the MOTOR makes. It cannot
see the far side of the gearbox. The scale can.

    slope of (measured torque vs commanded torque)  =  transmission efficiency
    x-intercept                                     =  static friction, free

RUN THIS YOURSELF so you see each step announced live:

    python3 characterize/lever_test.py                 # 4 -> 18 N·m in 2 N·m steps
    python3 characterize/lever_test.py 18 2            # max_Nm  step_Nm
    python3 characterize/lever_test.py 18 2 -1         # other direction

Write the scale reading next to each step number as it happens, then hand me the
list.

WHY IT COMMANDS TORQUE, NOT CURRENT
  set_current() in this moteus build takes only (d_A, q_A, query), it CANNOT
  arm the watchdog. Kill such a script and the controller keeps driving; that is
  exactly how 2.5 A got left running into the scale on 2026-08-30.
  set_position() with zeroed gains takes watchdog_timeout, so an interrupt stops
  the motor in 0.5 s. It also takes ilimit_scale, which properly kills the
  position integrator, a cleaner fix than remembering to set_stop() first.

SETUP
  - Arm HORIZONTAL, far end on the scale through a POINT contact
  - Do NOT tare. Note the resting reading, then the loaded reading, and subtract.
    Taring under load then unloading leaves the zero drifting.
  - ARM_M is measured axis -> contact point. It multiplies into every result.
"""
import asyncio, csv, datetime, math, os, sys, time

MAX_TQ    = float(sys.argv[1]) if len(sys.argv) > 1 else 18.0
STEP_TQ   = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
DIRECTION = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
START_TQ  = float(sys.argv[4]) if len(sys.argv) > 4 else None  # first level; default STEP_TQ

ARM_M      = 0.389721
SCALE_KG   = 5.0
HOLD_S     = 20.0
SETTLE_S   = 3.0
PERIOD     = 0.002
WATCHDOG_S = 0.5
HARD_TQ    = 19.0          # 19 N·m / 0.3897 m = 4.97 kg, at the scale's ceiling
ABORT_FET_C   = 70.0
POS_GUARD_DEG = 15.0       # total travel budget: settle + scale flex. 15 deg = 10 cm at the tip.
VEL_GUARD_RPS = 0.10       # sustained speed only a free-swinging arm can reach
VEL_GUARD_N   = 50         # consecutive samples (100 ms) above it before aborting

LIMITS = {96,97,98,99,100,101,102,103}
FAULTS = {32:"calibration",33:"MOTOR DRIVER FAULT",34:"over voltage",35:"encoder",
          36:"motor not configured",37:"pwm overrun",38:"over temperature",
          39:"outside limit",40:"UNDER VOLTAGE",41:"config changed",42:"theta invalid",
          43:"position invalid",44:"driver enable",46:"timing violation",
          49:"position control error",50:"velocity control error"}


async def main():
    import moteus
    if MAX_TQ > HARD_TQ:
        print(f"refusing {MAX_TQ} N·m: that is {MAX_TQ/ARM_M/9.81:.1f} kg on a "
              f"{SCALE_KG:.0f} kg scale")
        return 1

    levels = []
    t = START_TQ if START_TQ is not None else STEP_TQ
    while t <= MAX_TQ + 1e-9:
        levels.append(round(t, 2)); t += STEP_TQ

    print(f"arm {ARM_M*1000:.3f} mm   direction {'+' if DIRECTION>0 else '-'}")
    print(f"{len(levels)} steps x {HOLD_S:.0f} s = {len(levels)*HOLD_S/60:.1f} min")
    print("\nWrite down the scale reading at each step.\n")
    print(f"  {'step':>4} {'cmd N·m':>8} {'scale if 100% eff':>19}")
    for i, L in enumerate(levels):
        print(f"  {i+1:>4} {L:8.1f} {L/ARM_M/9.81:16.2f} kg")
    print("\n(right column is the ceiling, real readings will be LOWER by the")
    print(" friction offset. That offset is the point: its x-intercept is your")
    print(" static friction.)\n")

    qr = moteus.QueryResolution()
    for n in ["abs_position","motor_temperature","trajectory_complete",
              "rezero_state","home_state"]:
        if hasattr(qr,n): setattr(qr,n,moteus.IGNORE)
    qr.mode=moteus.INT8; qr.fault=moteus.INT8; qr.position=moteus.F32
    qr.velocity=moteus.F32; qr.torque=moteus.F32
    qr.q_current=moteus.F32; qr.d_current=moteus.F32
    qr.voltage=moteus.F32; qr.temperature=moteus.INT8
    c = moteus.Controller(id=1, query_resolution=qr); R = moteus.Register

    # set_stop FIRST: clears any stale position integrator before torque mode.
    await c.set_stop(); await asyncio.sleep(0.3)
    v0 = (await c.query()).values
    print(f"idle: bus {v0[R.VOLTAGE]:.2f} V, fet {v0[R.TEMPERATURE]:.0f} C, "
          f"fault {int(v0[R.FAULT])}")
    if int(v0[R.FAULT]) and int(v0[R.FAULT]) not in LIMITS:
        print("pre-existing fault, clear it first"); return 1
    input("\n>>> Arm resting on the scale, NOTE THE RESTING READING, "
          "then press ENTER: ")
    # home is captured HERE, after the prompt, so adjusting the arm while the
    # prompt sits open cannot bake an offset into the position guard.
    home = (await c.query()).values[R.POSITION]

    rows, results, aborted = [], [], None
    vel_hits = 0
    t_start = time.monotonic()
    try:
        for i, L in enumerate(levels):
            tq = DIRECTION * L
            print(f"\n  >>> STEP {i+1}/{len(levels)}: {L:.1f} N·m, "
                  f"hold {HOLD_S:.0f} s, READ THE SCALE", flush=True)
            t_lvl = time.monotonic(); acc = []; k = 0
            while time.monotonic() - t_lvl < HOLD_S:
                r = await c.set_position(
                        position=math.nan, velocity=0.0,
                        kp_scale=0.0, kd_scale=0.0, ilimit_scale=0.0,
                        feedforward_torque=tq, maximum_torque=abs(tq),
                        watchdog_timeout=WATCHDOG_S, query=True)
                v = r.values
                dpos = (v[R.POSITION] - home) * 360.0
                row = {"t_s": round(time.monotonic()-t_start,3), "step": i+1,
                       "cmd_Nm": L, "torque_moteus_Nm": v[R.TORQUE],
                       "iq_A": v[R.Q_CURRENT], "id_A": v[R.D_CURRENT],
                       "pos_deg": round(dpos,4), "vel_rps": v[R.VELOCITY],
                       "bus_V": v[R.VOLTAGE], "fet_C": v[R.TEMPERATURE],
                       "fault": int(v[R.FAULT])}
                rows.append(row)
                if time.monotonic()-t_lvl > SETTLE_S: acc.append(row)
                f = int(v[R.FAULT])
                if f and f not in LIMITS:
                    raise RuntimeError(f"fault {f} ({FAULTS.get(f,'?')})")
                if int(v[R.MODE]) == 1: raise RuntimeError("controller faulted")
                if abs(dpos) > POS_GUARD_DEG:
                    raise RuntimeError(f"arm moved {dpos:+.1f} deg total, it is "
                                       "not landing on the scale (check direction "
                                       "and contact point)")
                vel_hits = vel_hits + 1 if abs(v[R.VELOCITY]) > VEL_GUARD_RPS else 0
                if vel_hits >= VEL_GUARD_N:
                    raise RuntimeError(f"arm swinging at {v[R.VELOCITY]:+.2f} rev/s "
                                       "-- nothing is blocking it")
                if v[R.TEMPERATURE] > ABORT_FET_C:
                    raise RuntimeError(f"fet {v[R.TEMPERATURE]:.0f} C")
                k += 1
                s = t_lvl + k*PERIOD - time.monotonic()
                if s > 0: await asyncio.sleep(s)
            mt = sum(x["torque_moteus_Nm"] for x in acc)/len(acc)
            iq = sum(x["iq_A"] for x in acc)/len(acc)
            results.append({"step":i+1,"cmd_Nm":L,"moteus_Nm":round(mt,3),
                            "iq_A":round(iq,3),"fet_C":acc[-1]["fet_C"],
                            "drift_deg":acc[-1]["pos_deg"]})
            print(f"      moteus {mt:6.2f} N·m   Iq {iq:5.2f} A   "
                  f"fet {acc[-1]['fet_C']:.0f}C   drift {acc[-1]['pos_deg']:+5.2f} deg")
    except (Exception, KeyboardInterrupt) as e:
        # catch EVERYTHING: an unexpected error type must still fall through to
        # the CSV write below, or the run's data is silently lost.
        aborted = str(e) or type(e).__name__ or "interrupted"
        print(f"\nSTOP: {aborted}\n")
    finally:
        try: await c.set_stop()
        except Exception: pass
        print("\nstopped. motor is off.")

    if rows:
        os.makedirs("data", exist_ok=True)
        stamp = f"{datetime.datetime.now():%Y%m%d_%H%M%S}"
        with open(f"data/lever_{stamp}.csv","w",newline="") as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
        with open(f"data/lever_{stamp}_steps.csv","w",newline="") as f:
            w=csv.DictWriter(f,fieldnames=list(results[0].keys())); w.writeheader(); w.writerows(results)
        print(f"-> data/lever_{stamp}.csv  and  _steps.csv")
    print("\nGive me the scale readings, one per step, and the resting value.")
    return 0

sys.exit(asyncio.run(main()))
