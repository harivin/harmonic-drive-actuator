#!/usr/bin/env python3
"""Move the lever arm a controlled amount, slowly, torque-limited.

Used to walk the arm from vertical down onto the scale WITHOUT the possibility of
overloading it. The torque limit IS the scale limit:

    max scale force = MAX_TQ / ARM_M      13 N·m / 0.389721 m = 33 N = 3.4 kg

so even a full overshoot into the scale cannot exceed 3.4 kg on a 5 kg scale.
Velocity is kept low so there is no impact transient either.

  python3 characterize/approach.py [deg] [max_torque] [vel_rps]
  python3 characterize/approach.py 10           # small direction-check move
  python3 characterize/approach.py -90          # negative = the other way
"""
import asyncio, math, sys, time

DEG    = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
MAX_TQ = float(sys.argv[2]) if len(sys.argv) > 2 else 13.0
VEL    = float(sys.argv[3]) if len(sys.argv) > 3 else 0.02

ARM_M      = 0.389721
SCALE_CAP  = 5.0
HARD_TQ    = 15.0     # above this the scale could be overloaded at this arm length


async def main():
    import moteus
    if MAX_TQ > HARD_TQ:
        print(f"refusing {MAX_TQ} N·m: at {ARM_M*1000:.1f} mm that is "
              f"{MAX_TQ/ARM_M/9.81:.1f} kg on a {SCALE_CAP:.0f} kg scale")
        return 1
    kg = MAX_TQ / ARM_M / 9.81
    print(f"move {DEG:+.1f} deg at {VEL} rev/s, torque limit {MAX_TQ} N·m")
    print(f"  -> the arm cannot push harder than {kg:.1f} kg on the scale")
    print()

    qr = moteus.QueryResolution()
    for n in ["abs_position","motor_temperature","trajectory_complete",
              "rezero_state","home_state"]:
        if hasattr(qr,n): setattr(qr,n,moteus.IGNORE)
    qr.mode=moteus.INT8; qr.fault=moteus.INT8; qr.position=moteus.F32
    qr.velocity=moteus.F32; qr.torque=moteus.F32; qr.q_current=moteus.F32
    qr.voltage=moteus.F32; qr.temperature=moteus.INT8
    c = moteus.Controller(id=1, query_resolution=qr); R = moteus.Register

    await c.set_stop(); await asyncio.sleep(0.3)
    home = (await c.query()).values[R.POSITION]
    target = home + DEG/360.0
    print(f"start {home*360:.2f} deg  ->  target {target*360:.2f} deg\n")

    t0=time.monotonic(); k=0; last=0.0
    try:
        while time.monotonic()-t0 < abs(DEG)/360.0/VEL + 8.0:
            r = await c.set_position(position=target, velocity=0.0,
                                     velocity_limit=VEL, accel_limit=0.2,
                                     maximum_torque=MAX_TQ,
                                     watchdog_timeout=0.5, query=True)
            v=r.values
            pos=(v[R.POSITION]-home)*360.0
            f=int(v[R.FAULT])
            if f and f not in (96,97,98,99,100,101,102,103):
                print(f"FAULT {f}"); break
            if time.monotonic()-t0-last > 1.0:
                last=time.monotonic()-t0
                print(f"  t {last:4.1f}s  moved {pos:+7.2f} deg  "
                      f"torque {v[R.TORQUE]:6.2f} N·m  "
                      f"= {abs(v[R.TORQUE])/ARM_M/9.81:4.1f} kg at the arm tip")
            if abs(pos-DEG) < 0.15 and abs(v[R.VELOCITY]) < 0.002:
                print(f"\n  arrived: {pos:+.2f} deg"); break
            k+=1
            s=t0+k*0.002-time.monotonic()
            if s>0: await asyncio.sleep(s)
    finally:
        await c.set_stop()
        v=(await c.query()).values
        print(f"\nstopped. final {(v[R.POSITION]-home)*360:+.2f} deg from start")
        print("Gravity torque on the arm is ~0.4 N·m against 9-12 N·m of gearbox")
        print("friction, so it stays where it is with the controller off.")
asyncio.run(main())
