#!/usr/bin/env python3
"""
First power-on of the ASSEMBLED gearbox in closed-loop current mode.

Context: the input ring magnet was just reinstalled, restoring commutation.
The output shaft magnet is still not installed (no output-side position
sensing). This gearbox has never turned under power, the two voltage_foc
attempts (data/vfoc_*.csv) drew current but neither one produced confirmed
rotation, and open-loop mode has no real current limit anyway. This is the
actual first test with real commutation and real current limiting.

Standing rule on this rig: always start a new test at the lowest torque and
speed that could work, then step up. So this is NOT the 8-step, 10 N·m sweep used
motor-only with nothing attached, it's a single conservative step:

  - ONE speed: 0.10 output rev/s (0.05 is known to stall on stiction, no
    point repeating that here)
  - torque limit dropped to 2.0 N·m output (vs 10 N·m used with nothing
    attached), about 10% of rated (21 N·m), far under peak (44 N·m)
  - forward direction only
  - short dwell, just enough to confirm smooth rotation and get one clean
    CRC reading under real current (2-3 A range, never validated before -
    every prior clean CRC result was at <1 A)

Reuses characterize/speed_sweep.py's tested query verification, abort logic, and
AuxReader rather than re-implementing them, only the sweep PARAMETERS are
overridden here.

RUN WITH:  /opt/anaconda3/bin/python3 bringup/first_power_on.py
"""

import asyncio
import datetime
import os
import sys

import actuator as base

# this one reuses the speed sweep itself, not just the shared library, so the
# repo root has to be importable whether you run it as a script or installed
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import characterize.speed_sweep as sweep

# --- override the sweep parameters for this first, conservative step -------
sweep.SPEEDS_RPS_OUT = [0.20]
sweep.MAX_TORQUE_NM_OUT = 10.0
sweep.DWELL_S = 5.0
sweep.SETTLE_S = 1.0
DIRECTIONS = [(+1.0, "forward")]


async def main():
    import moteus

    c = moteus.Controller(id=sweep.CAN_ID,
                          query_resolution=base.make_query_resolution(moteus))

    pre = await c.query()
    miss = base.verify_query(moteus, pre)
    if miss:
        print(f"*** query truncated, missing {miss}, aborting", file=sys.stderr)
        return 1
    p = base.unpack(moteus, pre)
    if p["fault"]:
        print(f"*** pre-existing fault {p['fault']}, diagnose first", file=sys.stderr)
        return 1

    print("=" * 72)
    print("FIRST POWER-ON: assembled gearbox, closed-loop current mode")
    print("=" * 72)
    print("  This is the first time this gearbox has turned under real")
    print("  commutation and real current limiting. Output magnet is still")
    print("  not installed, no output-side position feedback.")
    print()
    print(f"  speed          {sweep.SPEEDS_RPS_OUT[0]} output rev/s (single step)")
    print(f"  torque limit    {sweep.MAX_TORQUE_NM_OUT:g} N·m output "
          f"(~{100*sweep.MAX_TORQUE_NM_OUT/21:.0f}% of 21 N·m rated)")
    print(f"  direction       forward only")
    print(f"  dwell           {sweep.DWELL_S:g} s")
    print(f"  abort on        fault | fet>{base.ABORT_FET_TEMP_C:g}C | "
          f"|Iq|>{base.ABORT_Q_CURRENT_A:g}A | Vbus outside "
          f"[{base.ABORT_BUS_V_LOW:g},{base.ABORT_BUS_V_HIGH:g}]")
    print()
    print(f"  Bus {p['bus_voltage_V']:.2f} V, fet {p['fet_temp_C']:.1f} C, "
          f"fault {p['fault']}")
    print()
    print("  Watch the assembly the entire time. Hand on the PSU switch.")
    print("  If it doesn't turn smoothly, or anything sounds wrong, cut")
    print("  power yourself, don't wait for the script.")
    print()

    try:
        resp = input("Type GO to start, anything else to abort: ").strip()
    except (EOFError, KeyboardInterrupt):
        resp = ""
    if resp != "GO":
        print("Aborted, nothing commanded.")
        return 0

    os.makedirs("data", exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = f"data/first_power_on_{stamp}.csv"

    rows, aborted = await sweep.run_sweep(DIRECTIONS, csv_path)

    if rows:
        sweep.plot(rows, csv_path)

    if aborted:
        print(f"\n*** Run ended early: {aborted}")
        return 1

    print("\nDone. Ask the person who was watching: did it turn smoothly?")
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
