#!/usr/bin/env python3
"""Back-and-forth position moves, log running torque per pass.
Aborts on fault/temp/current (check_limits) AND on the Orbis warning
flag: if the wave gen walks out of the flexspline the readhead gap changes
and the Orbis flags it before anything else would."""
import asyncio, csv, datetime, math, os, statistics as st, sys, time
import actuator as base

SPAN_REV = 2.0; VEL = 0.20; ACCEL = 0.5; MAX_TQ = 15.0; KP_SCALE = 1.0
DURATION_S = float(sys.argv[1]) if len(sys.argv) > 1 else 600.0
FF = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
VEL = float(sys.argv[3]) if len(sys.argv) > 3 else VEL
PASS_TIMEOUT_S = SPAN_REV / VEL * 2.5   # if a pass takes 2.5x nominal, it's stuck

async def main():
    import moteus
    c = moteus.Controller(id=1, query_resolution=base.make_query_resolution(moteus))
    aux = base.AuxReader(moteus, c)
    await c.set_stop(); r = await c.query()
    if base.verify_query(moteus, r): print("query truncated"); return 1
    s0 = base.unpack(moteus, r)
    if s0["fault"]: print("pre-existing fault", s0["fault"]); return 1
    if not await aux.connect(): print("aux1 unavailable, no warning-flag canary, refusing"); return 1
    if aux.last_flat.get("bissc.warning_flag"):
        print("Orbis warning flag already set before start, fix gap first"); return 1
    home = s0["position_rev_out"]; targets = [home + SPAN_REV, home]
    print(f"run {DURATION_S:.0f}s: +-{SPAN_REV} rev at {VEL} rev/s, FF {FF} N·m, cap {MAX_TQ} N·m, bus {s0['bus_voltage_V']:.1f} V, fet {s0['fet_temp_C']:.0f} C")
    rows=[]; passes=[]; t0=time.monotonic(); aborted=None; k=0; per=0.02; nxt=0; warn=False; crc=float("nan")
    try:
        i=0; sp=home; direction=+1; pos=home; stuck_since=None; pass_t0=time.monotonic(); mv=[]
        tprev=time.monotonic()
        while time.monotonic()-t0 < DURATION_S:
            now=time.monotonic(); t=now-t0; dt=now-tprev; tprev=now
            # advancing setpoint; clamp error so catch-up stays bounded
            sp += direction*VEL*dt
            sp = max(min(sp, pos+2.0), pos-2.0)
            r = await c.set_position(position=sp, velocity=direction*VEL, velocity_limit=0.4, accel_limit=ACCEL,
                                     maximum_torque=MAX_TQ, kp_scale=KP_SCALE, feedforward_torque=direction*FF, watchdog_timeout=1.0, query=True)
            s_ = base.unpack(moteus, r); pos=s_["position_rev_out"]
            if t>=nxt:
                _,_,crc,_ = await aux.sample(); nxt=t+0.2
                warn = bool(aux.last_flat.get("bissc.warning_flag"))
            rows.append({"t_s":round(t,3),"pass":i+1,"setpoint":sp,**s_,"crc_delta":crc,"orbis_warn":int(warn)})
            base.check_limits(s_)
            if warn: raise base.Abort("ORBIS WARNING FLAG: readhead gap changed, wave gen may be moving")
            moving = abs(s_["velocity_rps_out"])>0.05
            if moving: mv.append(s_); stuck_since=None
            elif abs(s_["torque_Nm_out"])>=0.95*MAX_TQ:
                stuck_since = stuck_since or now
                if now-stuck_since>20: raise base.Abort(f"stuck at cap {MAX_TQ} N·m for 20s at pos {pos-home:+.3f} rev")
            # end of pass: reverse
            if (direction>0 and pos>=home+SPAN_REV) or (direction<0 and pos<=home):
                i+=1; pt=now-pass_t0
                if mv:
                    tq=[abs(x["torque_Nm_out"]) for x in mv]
                    passes.append({"pass":i,"t_s":round(t,1),"dir":"+" if direction>0 else "-","dur_s":round(pt,1),
                                   "tq_mean":st.mean(tq),"tq_p90":sorted(tq)[int(0.9*len(tq))],"vel_mean":st.mean(abs(x["velocity_rps_out"]) for x in mv),
                                   "fet":mv[-1]["fet_temp_C"],"crc":crc})
                    p_=passes[-1]
                    print(f"  pass {i:3d} {p_['dir']} t={p_['t_s']:5.0f}s  {p_['dur_s']:5.1f}s  tq mean {p_['tq_mean']:5.2f} p90 {p_['tq_p90']:5.2f} N·m  "
                          f"({p_['tq_mean']/50*1000:4.0f} mN·m input)  vel {p_['vel_mean']:.3f}  fet {p_['fet']:.0f}C  crc+{crc:.0f}", flush=True)
                direction=-direction; sp=pos; pass_t0=now; mv=[]
            k+=1; slp=t0+k*per-time.monotonic()
            if slp>0: await asyncio.sleep(slp)
    except base.Abort as e: aborted=str(e); print("ABORT:",e)
    except KeyboardInterrupt: aborted="interrupt"
    finally:
        await c.set_stop(); print("stopped")
    os.makedirs("data",exist_ok=True); stamp=f"{datetime.datetime.now():%Y%m%d_%H%M%S}"
    with open(f"data/run_in_{stamp}.csv","w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    if passes:
        with open(f"data/run_in_{stamp}_passes.csv","w",newline="") as f:
            w=csv.DictWriter(f,fieldnames=list(passes[0].keys())); w.writeheader(); w.writerows(passes)
    if len(passes)>=4:
        a=st.mean(p["tq_mean"] for p in passes[:2]); b=st.mean(p["tq_mean"] for p in passes[-2:])
        print(f"running torque first 2 passes {a:.2f} -> last 2 passes {b:.2f} N·m output-ref ({100*(b-a)/a:+.0f}%)")
    print(f"csv: data/run_in_{stamp}.csv  passes: data/run_in_{stamp}_passes.csv")
    return 1 if aborted else 0
sys.exit(asyncio.run(main()))
