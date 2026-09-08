#!/usr/bin/env python3
"""How stiff can this joint be rendered before it rings.

Impedance control over CAN-FD: tau = K*(theta_d - theta) - D*omega, computed in
Python each cycle and sent as pure torque (kp_scale=kd_scale=0, feedforward).
Steps the setpoint, sweeps K, records the response, detects oscillation, and
compares the onset to the discrete-time passivity bound K < 2*D/T (Colgate &
Hogan).

The answer on this build is that the bound is irrelevant: it rings about 10x
below where the loop rate says it should, because the flexspline's own torsional
resonance binds first.

Stiffness feedback here is the Orbis, on the ROTOR side, divided by 50. That
puts the gearbox dead band inside the loop, which is exactly why
control/impedance.py exists and closes on the output encoder instead. For a
step-response ladder it does not matter, and this is the run that produced
data/impedance_20260821_*.csv and the figure in analysis/.

    python3 characterize/impedance_sweep.py        # no friction feedforward
    python3 characterize/impedance_sweep.py 6      # with 6 N·m Coulomb comp
"""
import asyncio, csv, datetime, math, os, statistics as st, sys, time
import actuator as base

STEP_REV = 0.05                 # 18 deg step
KS = [100, 200, 400, 800, 1600, 3200]
J_REV = 2.0                     # est. output inertia, N·m/(rev/s^2) (rotor*2500 + arm)
ZETA = 0.5                      # D chosen for this damping ratio: D = 2*zeta*sqrt(K*J)
MAX_TQ = 15.0; POS_GUARD_REV = 0.5
HOLD_S = 2.5; OSC_WIN = 0.3; OSC_VEL_STD = 0.15
FF_FRICTION = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0   # optional Coulomb comp, N·m

async def step(c, moteus, K, D, rows, home):
    theta_d = home; t0 = time.monotonic(); k = 0; period = 0.001
    phase = "settle"; t_step = 1.0; osc = False; last_dt = []
    while True:
        now = time.monotonic(); t = now - t0
        if t > t_step + HOLD_S: break
        if t >= t_step and phase == "settle": phase = "step"; theta_d = home + STEP_REV
        # measure (use previous reply) -> compute -> command with query
        r = await c.set_position(position=math.nan, velocity=0.0, kp_scale=0.0, kd_scale=0.0,
                                 feedforward_torque=getattr(step, "tau", 0.0), maximum_torque=MAX_TQ,
                                 watchdog_timeout=0.5, query=True)
        s = base.unpack(moteus, r); th = s["position_rev_out"]; om = s["velocity_rps_out"]
        tau = K * (theta_d - th) - D * om
        if FF_FRICTION and abs(om) > 0.005: tau += FF_FRICTION * (1 if om > 0 else -1)
        tau = max(-MAX_TQ, min(MAX_TQ, tau)); step.tau = tau
        rows.append({"t_s": round(t - t_step, 4), "K": K, "D": round(D, 2), "phase": phase, "theta_d": theta_d,
                     "theta": th, "omega": om, "tau_cmd": tau, "tau_meas": s["torque_Nm_out"],
                     "q_current_A": s["q_current_A"], "fet": s["fet_temp_C"], "fault": s["fault"]})
        base.check_limits(s)
        if abs(th - home) > POS_GUARD_REV: raise base.Abort(f"position guard: {th-home:+.2f} rev from home")
        recent = [x["omega"] for x in rows[-int(OSC_WIN/period):] if x["K"] == K]
        if t > t_step + 0.5 and len(recent) > 50 and st.pstdev(recent) > OSC_VEL_STD:
            osc = True; break
        k += 1; slp = t0 + k * period - time.monotonic()
        if slp > 0: await asyncio.sleep(slp)
    step.tau = 0.0
    return osc

def analyze(seg, K, D, T):
    post = [x for x in seg if x["phase"] == "step"]
    if not post: return None
    th0 = seg[0]["theta"]; target = post[0]["theta_d"]; y = [(x["theta"] - th0) / STEP_REV for x in post]; t = [x["t_s"] for x in post]
    rise = next((t[i] for i, v in enumerate(y) if v >= 0.9), float("nan"))
    peak = max(y); overshoot = max(0.0, (peak - 1.0) * 100)
    tail = [x for x in post if x["t_s"] > HOLD_S - 0.5]
    ss_err_deg = st.mean((target - x["theta"]) for x in tail) * 360 if tail else float("nan")
    om_tail = [x["omega"] for x in tail]; ringing = st.pstdev(om_tail) if om_tail else float("nan")
    return {"K": K, "D": D, "rise_s": rise, "overshoot_pct": overshoot, "ss_err_deg": ss_err_deg,
            "tail_vel_std": ringing, "bound_2D_T": 2 * D / T, "K_over_bound": K / (2 * D / T)}

async def main():
    import moteus
    # loop period, p99. Measured 2026-08-16 on the CAN-FD query path with no
    # motion commanded; raw histogram in data/backemf_20260816_195848.csv.
    T = 0.00184
    c = moteus.Controller(id=1, query_resolution=base.make_query_resolution(moteus))
    await c.set_stop(); s0 = base.unpack(moteus, await c.query()); home = s0["position_rev_out"]
    print(f"impedance step test: {STEP_REV*360:.0f} deg step, zeta {ZETA}, J_est {J_REV} N·m/(rev/s^2), clamp {MAX_TQ} N·m, FF {FF_FRICTION} N·m, T={T*1000:.2f} ms")
    print(f"  {'K':>5} {'D':>6}  {'rise s':>6} {'ovr%':>5} {'ss err°':>7} {'tail v std':>10} {'2D/T':>7} {'K/bound':>7}  note")
    rows = []; res = []
    try:
        for K in KS:
            D = 2 * ZETA * math.sqrt(K * J_REV)
            n0 = len(rows)
            # return to home gently before each step (position mode, stored gains)
            for _ in range(150):
                await c.set_position(position=home, velocity=0.0, velocity_limit=0.3, accel_limit=0.5, maximum_torque=MAX_TQ, watchdog_timeout=0.5); await asyncio.sleep(0.01)
            await c.set_stop(); await asyncio.sleep(0.3)   # clears the position-loop integrator (ki/ilimit are NOT scaled by kp_scale)
            osc = await step(c, moteus, K, D, rows, home)
            if osc:
                await c.set_stop(); print(f"  {K:5.0f} {D:6.1f}  OSCILLATION -> stopped this K"); res.append({"K": K, "D": D, "osc": True, "bound_2D_T": 2*D/T, "K_over_bound": K/(2*D/T)}); await asyncio.sleep(0.5); continue
            a = analyze(rows[n0:], K, D, T); a["osc"] = False; res.append(a)
            print(f"  {K:5.0f} {D:6.1f}  {a['rise_s']:6.3f} {a['overshoot_pct']:5.1f} {a['ss_err_deg']:7.2f} {a['tail_vel_std']:10.4f} {a['bound_2D_T']:7.0f} {a['K_over_bound']:7.3f}")
    except base.Abort as e: print("ABORT:", e)
    finally: await c.set_stop(); print("stopped")
    os.makedirs("data", exist_ok=True); stamp = f"{datetime.datetime.now():%Y%m%d_%H%M%S}"
    with open(f"data/impedance_{stamp}.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    with open(f"data/impedance_{stamp}_summary.csv", "w", newline="") as f:
        keys = sorted({k for r in res for k in r}); w = csv.DictWriter(f, fieldnames=keys); w.writeheader(); w.writerows(res)
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    cols = ["#0072B2", "#D55E00", "#009E73", "#CC79A7", "#E69F00", "#56B4E9", "#999999"]
    fig, ax = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    for K, col in zip(KS, cols):
        seg = [x for x in rows if x["K"] == K and x["phase"] == "step"]
        if not seg: continue
        th0 = [x for x in rows if x["K"] == K][0]["theta"]
        ax[0].plot([x["t_s"] for x in seg], [(x["theta"] - th0) * 360 for x in seg], color=col, lw=1.5, label=f"K={K}")
        ax[1].plot([x["t_s"] for x in seg], [x["tau_cmd"] for x in seg], color=col, lw=1)
    ax[0].axhline(STEP_REV * 360, color="#999", ls="--", lw=1); ax[0].set_ylabel("output angle (deg)"); ax[0].legend(frameon=False, ncol=4, fontsize=9)
    ax[1].set_ylabel("commanded torque (N·m)"); ax[1].set_xlabel("s after step"); ax[1].set_ylim(-MAX_TQ*1.05, MAX_TQ*1.05)
    for a in ax: a.grid(True, color="#ddd")
    fig.suptitle(f"Impedance step response, {STEP_REV*360:.0f}° step, ζ={ZETA}, FF={FF_FRICTION} N·m, Python over CAN-FD"); fig.tight_layout()
    fig.savefig(f"data/impedance_{stamp}.png", dpi=140); print("plot:", f"data/impedance_{stamp}.png")
sys.exit(asyncio.run(main()))
