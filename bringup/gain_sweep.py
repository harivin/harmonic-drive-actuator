#!/usr/bin/env python3
"""Automated position-gain sweep. For each (kp,kd), run 1 rev out+back at VEL,
score smoothness from the Orbis-derived velocity, detect oscillation, pick best.
Gains are applied as scales on the stored config (kp 1000, kd 20)."""
import asyncio, csv, datetime, math, os, statistics as st, sys, time
import actuator as base
BASE_KP, BASE_KD = 8000.0, 160.0
GAINS = [(4000,80),(8000,160)]
VEL = float(sys.argv[1]) if len(sys.argv)>1 else 0.05
SPAN, MAX_TQ, ACCEL = 1.0, 15.0, 0.3
OSC_VEL_STD = 0.08   # rev/s: velocity std above this at a steady command = oscillating -> abort that gain

async def move(c,moteus,direction,kp,kd,rows):
    r=await c.query(); pos=base.unpack(moteus,r)["position_rev_out"]; sp=pos; tgt=pos+direction*SPAN
    t0=time.monotonic(); tprev=t0; k=0
    while True:
        now=time.monotonic(); dt=now-tprev; tprev=now; t=now-t0
        sp+=direction*VEL*dt; sp=max(min(sp,pos+1.0),pos-1.0)
        if (direction>0 and sp>tgt) or (direction<0 and sp<tgt): sp=tgt
        r=await c.set_position(position=sp,velocity=direction*VEL,velocity_limit=0.3,accel_limit=ACCEL,
                               maximum_torque=MAX_TQ,kp_scale=kp/BASE_KP,kd_scale=kd/BASE_KD,watchdog_timeout=1.0,query=True)
        s=base.unpack(moteus,r); pos=s["position_rev_out"]
        rows.append({"t_s":round(t,3),"kp":kp,"kd":kd,"dir":direction,"setpoint":sp,**s})
        base.check_limits(s)
        # oscillation guard on the last 0.5 s
        recent=[x["velocity_rps_out"] for x in rows[-25:] if x["kp"]==kp and x["dir"]==direction]
        if len(recent)>=25 and st.pstdev(recent)>OSC_VEL_STD: raise base.Abort(f"oscillation at kp {kp}: vel std {st.pstdev(recent):.3f}")
        if abs(pos-tgt)<0.01 and abs(s["velocity_rps_out"])<0.02 and abs(sp-tgt)<1e-9: break
        if t>SPAN/VEL*2.0: break
        k+=1; slp=t0+k*0.02-time.monotonic()
        if slp>0: await asyncio.sleep(slp)
    return t

async def main():
    import moteus
    c=moteus.Controller(id=1,query_resolution=base.make_query_resolution(moteus))
    await c.set_stop()
    print(f"gain sweep at {VEL} rev/s, {SPAN} rev out+back, cap {MAX_TQ} N·m")
    print(f"  {'kp':>5} {'kd':>4}  {'stall%':>6} {'vel_cov':>7} {'pos_err_rms':>11} {'tq_mean':>7} {'tq_max':>6}  verdict")
    rows=[]; res=[]
    try:
        for kp,kd in GAINS:
            n0=len(rows); ok=True
            try:
                for d in (+1,-1): await move(c,moteus,d,kp,kd,rows)
            except base.Abort as e:
                if "oscillation" in str(e): ok=False; print(f"  {kp:5.0f} {kd:4.0f}  OSCILLATED: {e}"); await c.set_stop(); await asyncio.sleep(0.5); continue
                raise
            seg=[x for x in rows[n0:]]
            mid=[x for x in seg if 0.15*SPAN/VEL < x["t_s"] < 0.85*SPAN/VEL]
            v=[abs(x["velocity_rps_out"]) for x in mid]; tq=[abs(x["torque_Nm_out"]) for x in mid]
            err=[abs(x["setpoint"]-x["position_rev_out"])*360 for x in mid]
            r_={"kp":kp,"kd":kd,"stall":100*sum(1 for x in v if x<VEL/2)/len(v),"cov":st.pstdev(v)/max(st.mean(v),1e-6),
                "err_rms_deg":math.sqrt(sum(e*e for e in err)/len(err)),"tq_mean":st.mean(tq),"tq_max":max(tq)}
            res.append(r_)
            print(f"  {kp:5.0f} {kd:4.0f}  {r_['stall']:5.0f}% {r_['cov']:7.2f} {r_['err_rms_deg']:10.2f}° {r_['tq_mean']:7.2f} {r_['tq_max']:6.1f}")
    finally:
        await c.set_stop(); print("stopped")
    os.makedirs("data",exist_ok=True); stamp=f"{datetime.datetime.now():%Y%m%d_%H%M%S}"
    with open(f"data/gain_sweep_{stamp}.csv","w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    if res:
        best=min(res,key=lambda r:(r["cov"],r["stall"]))
        print(f"\nBEST: kp {best['kp']:.0f} kd {best['kd']:.0f}  (vel cov {best['cov']:.2f}, stall {best['stall']:.0f}%, pos err {best['err_rms_deg']:.2f} deg rms)")
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig,ax=plt.subplots(len(GAINS),1,figsize=(10,2.4*len(GAINS)),sharex=True,squeeze=False); ax=ax[:,0]
    for a,(kp,kd) in zip(ax,GAINS):
        seg=[x for x in rows if x["kp"]==kp and x["dir"]>0]
        a.plot([x["t_s"] for x in seg],[x["velocity_rps_out"] for x in seg],color="#0072B2",lw=1)
        a.axhline(VEL,color="#999",ls="--",lw=1); a.set_ylabel(f"kp {kp}\nvel rev/s"); a.set_ylim(-0.02,max(0.12,VEL*2.5)); a.grid(True,color="#ddd")
    ax[-1].set_xlabel("s"); fig.suptitle(f"Position-gain sweep at {VEL} rev/s, velocity from the Orbis"); fig.tight_layout()
    fig.savefig(f"data/gain_sweep_{stamp}.png",dpi=140); print("plot:",f"data/gain_sweep_{stamp}.png")
sys.exit(asyncio.run(main()))
