#!/usr/bin/env python3
"""Position-mode move: lets kp build torque until breakaway, then runs at
velocity_limit. Unlike position=nan (kd-only), torque is not capped by
kd*vel_error, it rises to maximum_torque if the motor stalls."""
import asyncio, csv, datetime, math, os, sys, time
import actuator as base

DELTA_REV_OUT = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
VEL_LIMIT = 0.20
ACCEL_LIMIT = 0.5
MAX_TORQUE = 10.0
EXTRA_S = 12.0

async def main():
    import moteus
    c = moteus.Controller(id=1, query_resolution=base.make_query_resolution(moteus))
    aux = base.AuxReader(moteus, c)
    await c.set_stop()
    r = await c.query()
    if base.verify_query(moteus, r): print("query truncated"); return 1
    s = base.unpack(moteus, r)
    if s["fault"]: print("pre-existing fault", s["fault"]); return 1
    start_pos = s["position_rev_out"]; target = start_pos + DELTA_REV_OUT
    dur = abs(DELTA_REV_OUT)/VEL_LIMIT + EXTRA_S
    await aux.connect()
    print(f"move {start_pos:+.4f} -> {target:+.4f} out-rev at {VEL_LIMIT} rev/s, cap {MAX_TORQUE} N·m, ~{dur:.0f}s")
    rows=[]; t0=time.monotonic(); k=0; per=0.02; nxt=0; crc=float('nan'); aborted=None
    try:
        while True:
            t=time.monotonic()-t0
            if t>dur: break
            r = await c.set_position(position=target, velocity=0.0, velocity_limit=VEL_LIMIT,
                                     accel_limit=ACCEL_LIMIT, maximum_torque=MAX_TORQUE, query=True)
            s = base.unpack(moteus, r)
            if aux.available and t>=nxt:
                _,_,crc,_ = await aux.sample(); nxt=t+0.2
            rows.append({"t_s":round(t,3), **s, "crc_delta":crc})
            base.check_limits(s)
            if k%50==0:
                lim = base.MOTEUS_LIMIT_REASONS.get(s["fault"],"")
                print(f"  t={t:4.1f} pos={s['position_rev_out']-start_pos:+.4f} vel={s['velocity_rps_out']:+.3f} "
                      f"tq={s['torque_Nm_out']:+.2f} Iq={s['q_current_A']:+.2f} fet={s['fet_temp_C']:.0f} crc+{crc:.0f} {lim}")
            k+=1; slp=t0+k*per-time.monotonic()
            if slp>0: await asyncio.sleep(slp)
    except base.Abort as e: aborted=str(e); print("ABORT:",e)
    finally:
        await c.set_stop(); print("stopped")
    os.makedirs("data",exist_ok=True)
    p=f"data/position_move_{datetime.datetime.now():%Y%m%d_%H%M%S}.csv"
    with open(p,"w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    moved = rows[-1]["position_rev_out"]-start_pos
    mv=[x for x in rows if abs(x["velocity_rps_out"])>0.05]
    print(f"moved {moved:+.4f} out-rev ({moved*360:+.1f} deg out, {moved*50:+.1f} rotor rev)")
    if mv:
        import statistics as st
        print(f"while moving: mean vel {st.mean(x['velocity_rps_out'] for x in mv):+.3f} rev/s, "
              f"mean torque {st.mean(x['torque_Nm_out'] for x in mv):+.2f} N·m, "
              f"mean Iq {st.mean(x['q_current_A'] for x in mv):+.2f} A, n={len(mv)}")
    print("csv:",p)
    return 1 if aborted else 0

sys.exit(asyncio.run(main()))
