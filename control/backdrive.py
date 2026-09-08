#!/usr/bin/env python3
"""Virtual backdrivability with measured friction + ripple comp.

Cancels three measured things (all from 2026-08-30/31 sweeps, current build):
  1. Coulomb friction  f0 = 4.02 N·m           (velocity-sign, tanh taper)
  2. 1x/rotor-rev ripple, 1.3 N·m amplitude     (rotor-phase-indexed)
  3. 2x/rotor-rev wave-generator ripple, 0.8 N·m (the 3.6-deg "steps")

    tau = SCALE * fric(rotor_phase) * tanh(om/W0)  -  D*om

Rotor phase comes from output POSITION mod 1/50 rev (Orbis is absolute per
rotor rev, so the phase is stable across runs).

    python3 control/backdrive.py               # SCALE=0.8, D=6, 60 s
    python3 control/backdrive.py 0.9 6 120     # scale D duration_s
    python3 control/backdrive.py 0.8 6 60 1    # 4th arg nonzero: no ripple terms
                                            # (A/B test, Coulomb comp only)

PUSH THE ARM BY HAND. Compare texture with/without ripple comp (4th arg).
Ctrl-C stops; watchdog 0.5 s armed on every command.

Logs to data/transparent_v2_*.csv. That prefix predates the rename of this file
and is left alone so the filenames still match my own archive.
"""
import asyncio, csv, datetime, math, sys, time

SCALE  = float(sys.argv[1]) if len(sys.argv) > 1 else 0.8
D      = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0
DUR    = float(sys.argv[3]) if len(sys.argv) > 3 else 60.0
NO_RIP = bool(len(sys.argv) > 4 and float(sys.argv[4]) != 0)
DITHER_NM = float(sys.argv[5]) if len(sys.argv) > 5 else 1.5   # anti-detent buzz
DITHER_HZ = 12.0
DET_SCALE = float(sys.argv[6]) if len(sys.argv) > 6 else 1.0   # detent comp multiplier

# friction magnitude vs ABSOLUTE rotor phase ((POSITION*50) mod 1), N·m:
# [f0, sin(1x), cos(1x), sin(2x), cos(2x)]  fit 2026-08-31 (absolute frame)
FRIC = [4.1364, 0.8021, 0.9766, 0.5987, -0.6532]
# conservative detent torque (direction-average) in the same absolute frame --
# applied ALWAYS-ON (a detent acts at zero velocity, so it must not be gated).
DET  = [0.0, -0.3213, 0.2221, 0.0955, 0.2534]

W0         = 0.015         # rev/s comp taper
MAX_TQ     = 10.0
GUARD_REV  = 0.5           # +-180 deg free-roam from start
V_RUNAWAY  = 0.30          # rev/s: if comp ever self-drives, brake hard
WATCHDOG_S = 0.5
PERIOD     = 0.001
LIMITS = {96,97,98,99,100,101,102,103}

def det_tq(phase):
    if NO_RIP: return 0.0
    return (DET[1]*math.sin(2*math.pi*phase) + DET[2]*math.cos(2*math.pi*phase)
            + DET[3]*math.sin(4*math.pi*phase) + DET[4]*math.cos(4*math.pi*phase))

def fric_mag(phase):
    f = (FRIC[0]
         + FRIC[1]*math.sin(2*math.pi*phase) + FRIC[2]*math.cos(2*math.pi*phase)
         + FRIC[3]*math.sin(4*math.pi*phase) + FRIC[4]*math.cos(4*math.pi*phase))
    return max(0.5, f) if not NO_RIP else FRIC[0]

async def main():
    import moteus
    qr = moteus.QueryResolution()
    qr.mode=moteus.INT8; qr.fault=moteus.INT8; qr.position=moteus.F32
    qr.velocity=moteus.F32; qr.torque=moteus.F32; qr.temperature=moteus.INT8
    qr._extra = {moteus.Register.ENCODER_1_POSITION: moteus.F32}
    c = moteus.Controller(id=1, query_resolution=qr); R = moteus.Register

    await c.set_stop(); await asyncio.sleep(0.3)
    v0 = (await c.query()).values
    if int(v0[R.FAULT]) and int(v0[R.FAULT]) not in LIMITS:
        print(f"pre-existing fault {int(v0[R.FAULT])}"); return 1
    home = v0[R.POSITION]
    print(f"transparent v2: scale {SCALE}, D {D}, "
          f"{'COULOMB ONLY (no ripple comp)' if NO_RIP else 'full comp (f0 + 1x + 2x ripple)'}")
    print(f"comp range {SCALE*min(fric_mag(p/100) for p in range(100)):.1f}-"
          f"{SCALE*max(fric_mag(p/100) for p in range(100)):.1f} N·m, {DUR:.0f} s. Push it.\n")

    import os; os.makedirs("data", exist_ok=True)
    stamp = f"{datetime.datetime.now():%Y%m%d_%H%M%S}"
    fn = f"data/transparent_v2_{stamp}.csv"
    FIELDS = ["t_s","pos_deg","vel_rps","tau_cmd","rotor_phase","e1_deg","fault"]
    csvf = open(fn,"w",newline=""); writer = csv.DictWriter(csvf,fieldnames=FIELDS)
    writer.writeheader()

    tau = 0.0; t0 = time.monotonic(); k = 0; last = 0.0
    travel = 0.0; prev = home; maxv = 0.0; guard_hits = 0
    try:
        while time.monotonic() - t0 < DUR:
            r = await c.set_position(position=math.nan, velocity=0.0,
                    kp_scale=0.0, kd_scale=0.0, ilimit_scale=0.0,
                    feedforward_torque=tau, maximum_torque=MAX_TQ,
                    watchdog_timeout=WATCHDOG_S, query=True)
            v = r.values; t = time.monotonic() - t0
            th = v[R.POSITION]; om = v[R.VELOCITY]
            phase = (th * 50.0) % 1.0
            if abs(om) > V_RUNAWAY:
                tau = -2*D*om; guard_hits += 1
            else:
                tau = (SCALE * fric_mag(phase) * math.tanh(om / W0) - D * om
                       + DET_SCALE * det_tq(phase)
                       + DITHER_NM * math.sin(2*math.pi*DITHER_HZ*t))
            tau = max(-MAX_TQ, min(MAX_TQ, tau))
            writer.writerow({"t_s": round(t,4), "pos_deg": round((th-home)*360,3),
                             "vel_rps": om, "tau_cmd": round(tau,3),
                             "rotor_phase": round(phase,4),
                             "e1_deg": round(v[R.ENCODER_1_POSITION]*360,3),
                             "fault": int(v[R.FAULT])})
            if k % 500 == 0: csvf.flush()
            f = int(v[R.FAULT])
            if f and f not in LIMITS: raise RuntimeError(f"fault {f}")
            if int(v[R.MODE]) in (1, 11): raise RuntimeError(f"mode {int(v[R.MODE])}")
            if abs(th - home) > GUARD_REV: raise RuntimeError(f"position guard {th-home:+.2f} rev")
            travel += abs(th - prev); prev = th; maxv = max(maxv, abs(om))
            if t - last > 5.0:
                print(f"  t={t:4.0f}s  pos {(th-home)*360:+7.1f} deg  travel {travel*360:5.0f} deg",
                      flush=True); last = t
            k += 1
            s = t0 + k*PERIOD - time.monotonic()
            if s > 0: await asyncio.sleep(s)
    except (Exception, KeyboardInterrupt) as e:
        print(f"\nSTOP: {e or type(e).__name__}")
    finally:
        try: await c.set_stop()
        except Exception: pass
        print(f"stopped. travel {travel*360:.0f} deg, peak vel {maxv:.2f} rev/s, "
              f"runaway brakes {guard_hits}")
    try: csvf.flush(); csvf.close()
    except Exception: pass
    print("->", fn)
    return 0

sys.exit(asyncio.run(main()))
