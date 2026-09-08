#!/usr/bin/env python3
"""Friction feedforward A/B: slow position moves with feedforward_torque =
FF*sign(direction). Measures stall fraction and velocity raggedness."""
import asyncio, csv, datetime, math, os, statistics as st, sys, time
import actuator as base
FFS=[0.0]; VEL=0.05; SPAN=1.0; MAX_TQ=15.0; KP=125.0; KD=200.0; ACCEL=0.3
GAINS=[(1.0,1.0)]

async def move(c,moteus,aux,home,direction,ff,rows,tag):
    global HOME; HOME=home
    r=await c.query(); pos=base.unpack(moteus,r)["position_rev_out"]; sp=pos; tgt=pos+direction*SPAN
    t0=time.monotonic(); tprev=t0; k=0
    while True:
        now=time.monotonic(); dt=now-tprev; tprev=now; t=now-t0
        sp+=direction*VEL*dt; sp=max(min(sp,pos+1.5),pos-1.5)
        if (direction>0 and sp>tgt) or (direction<0 and sp<tgt): sp=tgt
        r=await c.set_position(position=sp,velocity=direction*VEL,velocity_limit=0.3,accel_limit=ACCEL,
                               maximum_torque=MAX_TQ,kp_scale=KP,kd_scale=KD,feedforward_torque=direction*ff,watchdog_timeout=1.0,query=True)
        s=base.unpack(moteus,r); pos=s["position_rev_out"]
        rows.append({"t_s":round(t,3),"ff":ff,"kp":KP,"dir":direction,"setpoint":sp,**s})
        base.check_limits(s)
        if abs(pos-tgt)<0.01 and abs(s["velocity_rps_out"])<0.02 and abs(sp-tgt)<1e-9: break
        if t>SPAN/VEL*2.0: print(f"  [{tag}: did not finish in {t:.0f}s, pos {pos-HOME:+.2f}]"); break
        k+=1; slp=t0+k*0.02-time.monotonic()
        if slp>0: await asyncio.sleep(slp)
    return t

async def main():
    import moteus
    c=moteus.Controller(id=1,query_resolution=base.make_query_resolution(moteus)); aux=base.AuxReader(moteus,c)
    await c.set_stop(); s0=base.unpack(moteus,await c.query()); home=s0["position_rev_out"]
    print(f"feedforward A/B at {VEL} rev/s, {SPAN} rev out+back, cap {MAX_TQ}: FF = {FFS}")
    rows=[]; res=[]
    try:
        for (kp_,kd_) in GAINS:
            global KP,KD; KP,KD=kp_,kd_; ff=0.0; print(f"  kp {4*KP:.0f} N·m/rev, kd {0.05*KD:.1f} N·m/(rev/s)")
            for d in (+1,-1):
                n0=len(rows); dur=await move(c,moteus,aux,home,d,ff,rows,f"ff{ff} dir{d}")
                seg=rows[n0:]; mid=[x for x in seg if 0.15*dur<x["t_s"]<0.85*dur]  # steady portion
                v=[x["velocity_rps_out"]*d for x in mid]; tq=[x["torque_Nm_out"]*d for x in mid]
                stall=sum(1 for x in v if x<0.01)/len(v); cov=st.pstdev(v)/max(st.mean(v),1e-6)
                res.append({"ff":ff,"dir":d,"dur_s":round(dur,1),"stall_frac":stall,"vel_mean":st.mean(v),"vel_cov":cov,"tq_mean":st.mean(tq),"tq_max":max(tq)})
                print(f"  FF {ff:4.1f} {'+' if d>0 else '-'}  {dur:5.1f}s  stalled {100*stall:4.0f}%  vel {st.mean(v):.3f} cov {cov:4.2f}  tq mean {st.mean(tq):5.2f} max {max(tq):5.2f}",flush=True)
    except base.Abort as e: print("ABORT:",e)
    finally: await c.set_stop(); print("stopped")
    os.makedirs("data",exist_ok=True); stamp=f"{datetime.datetime.now():%Y%m%d_%H%M%S}"
    with open(f"data/ff_test_{stamp}.csv","w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig,ax=plt.subplots(len(GAINS),1,figsize=(10,2.6*len(GAINS)),sharex=True)
    for a,(kp_,kd_) in zip(ax,GAINS):
        seg=[x for x in rows if x["kp"]==kp_ and x["dir"]>0]; ff=kp_
        a.plot([x["t_s"] for x in seg],[x["velocity_rps_out"] for x in seg],color="#0072B2",lw=1)
        a.axhline(VEL,color="#999",ls="--",lw=1); a.set_ylabel(f"kp={4*ff:.0f}\nvel (rev/s)"); a.set_ylim(-0.02,0.32); a.grid(True,color="#ddd")
    ax[-1].set_xlabel("s"); fig.suptitle(f"Position-loop stiffness vs stick-slip, {VEL} rev/s forward moves, 15 N·m cap, no feedforward"); fig.tight_layout()
    fig.savefig(f"data/ff_test_{stamp}.png",dpi=140); print("plot:",f"data/ff_test_{stamp}.png")
sys.exit(asyncio.run(main()))
