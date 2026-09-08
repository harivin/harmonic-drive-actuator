#!/usr/bin/env python3
"""Measure the output magnet's disturbance to the Orbis.

Must run when output disconnected from the gearbox, so the
magnet can rotate while the ROTOR STAYS STILL. Any Orbis change is then pure
disturbance, because the rotor did not move. Assembled, turning the output turns
the rotor 50x with it and the two effects cannot be separated.

Nothing is commanded. No current, no motion. Pure logging, you turn it by hand.

  Phase 1 (3 s)  everything still  -> baseline Orbis rest jitter
  Phase 2 (N s)  you rotate the magnet slowly through a full turn -> disturbance

Run it twice and compare: once with the washer OFF, once with it at the candidate
shaft position. The delta is the washer's effect, measured instead of argued.

  python3 bringup/magnet_check.py washer_off 30
  python3 bringup/magnet_check.py washer_shaft 30

Pass criterion: swing <= +-60 counts is fine, +-100 costs ~15%
current, +-150 will not break away.
"""
import asyncio, csv, datetime, os, subprocess, json, sys, time
import statistics as st

LABEL = sys.argv[1] if len(sys.argv) > 1 else "run"
SPIN_S = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
STILL_S = 8.0
CPR = 16384


def as5047():
    try:
        r = subprocess.run(['/opt/anaconda3/bin/python3', '-m', 'moteus.moteus_tool',
                            '-t', '1', '--read', 'aux1'], capture_output=True, text=True)
        d = json.loads(r.stdout)
        return d['spi']['value'], d['bissc']['crc_errors'], \
               d['bissc']['error_flag'], d['bissc']['warning_flag']
    except Exception:
        return None, None, None, None


async def main():
    import moteus
    qr = moteus.QueryResolution()
    for n in ['torque', 'q_current', 'd_current', 'power', 'abs_position',
              'motor_temperature', 'trajectory_complete', 'rezero_state',
              'home_state', 'voltage', 'temperature']:
        if hasattr(qr, n):
            setattr(qr, n, moteus.IGNORE)
    qr.mode = moteus.INT8
    qr.fault = moteus.INT8
    qr.position = moteus.F32
    qr._extra = {moteus.Register.ENCODER_0_POSITION: moteus.F32}
    c = moteus.Controller(id=1, query_resolution=qr)
    R = moteus.Register

    spi0, crc0, ef0, wf0 = as5047()
    print(f"AS5047P before: {spi0}   Orbis crc {crc0}  error {ef0}  warning {wf0}")
    print()

    async def sample(dur, tag):
        out = []
        t0 = time.monotonic(); k = 0
        while time.monotonic() - t0 < dur:
            v = (await c.query()).values
            out.append((round(time.monotonic() - t0, 4),
                        v[R.ENCODER_0_POSITION], v[R.POSITION], int(v[R.FAULT])))
            k += 1
            s = t0 + k * 0.001 - time.monotonic()
            if s > 0:
                await asyncio.sleep(s)
        return out

    print(f"PHASE 1: hold everything STILL for {STILL_S:.0f} s (baseline) ...")
    still = await sample(STILL_S, "still")

    print()
    print(f"PHASE 2: NOW turn the magnet slowly through a FULL revolution, {SPIN_S:.0f} s.")
    print("           Do NOT touch the rotor or the motor.")
    print()
    spin = await sample(SPIN_S, "spin")

    spi1, crc1, ef1, wf1 = as5047()

    def report(rows, name):
        e = [r[1] for r in rows]
        rng = max(e) - min(e)
        print(f"  {name:12} n={len(e):6d}  raw delta {rng:.6e} rev")
        print(f"  {'':12} if source is ROTOR-referred : {rng*CPR:8.1f} counts")
        print(f"  {'':12} if source is OUTPUT-referred: {rng/0.02*CPR:8.1f} counts")
        return rng

    print()
    print("RESULTS")
    r_still = report(still, "still")
    r_spin = report(spin, "rotating")
    print()
    print(f"  AS5047P  {spi0} -> {spi1}   (it reads the magnet, so it SHOULD move a lot)")
    print(f"  Orbis crc_errors {crc0} -> {crc1}   growth {(crc1 or 0)-(crc0 or 0)}")
    print(f"  error_flag {ef0} -> {ef1}    warning_flag {wf0} -> {wf1}")
    print()
    print("  The disturbance is (rotating spread) with (still spread) as the noise floor.")
    print("  Target: <= ~120 counts peak-to-peak (i.e. +-60).")

    os.makedirs("data", exist_ok=True)
    stamp = f"{datetime.datetime.now():%Y%m%d_%H%M%S}"
    path = f"data/magnet_check_{LABEL}_{stamp}.csv"
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["phase", "t_s", "encoder0", "position", "fault"])
        for r in still: w.writerow(["still", *r])
        for r in spin:  w.writerow(["spin", *r])
    print(f"\n  -> {path}")

asyncio.run(main())
