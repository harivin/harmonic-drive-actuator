#!/usr/bin/env python3
"""Impedance control with OUTPUT-side position feedback.

The first version of this (2026-08-21) closed the loop on the Orbis, which
watches the ROTOR, and divided through by 50. That put the gearbox dead band
INSIDE the loop: torque below breakaway moved the rotor but not the output, and
the controller could not tell the difference. This one closes the stiffness term
on the AS5047P, which watches the actual output:

    tau = K*(theta_d - theta_OUT) - D*omega + FF*sign(omega)

  theta_OUT : AS5047P (ENCODER_1), sign-corrected (-1 vs POSITION, verified
              2026-08-30: ratio -1.0001 over 20 deg)
  omega     : moteus R.VELOCITY (Orbis-derived, 50x finer, the AS5047P's own
              velocity estimate is +-0.5 rev/s noise, unusable for damping)
  FF        : Coulomb friction comp, default 4.0 N·m (measured kinetic friction
              4.1-4.4 N·m at 0.05 rev/s, 2026-08-30)

    python3 control/impedance.py 150 10 2.5     # SETTLED "great" config (2026-08-31)
    python3 control/impedance.py 400 16 2.5     # stiffer: tight tracking, sub-degree
    python3 control/impedance.py 100 8 2.5      # softest usable; parks ~0.5-2 deg
    # args: K D FF DUR KI. Comp-velocity LP coeff is 0.08 (~11 Hz): 0.03 lagged
    # (sluggish), 0.15 passed sensor noise (choppy), 0.08 is the balance point.

Holds the CURRENT position. Push the arm and feel it. Ctrl-C stops (watchdog
armed, 0.5 s).

Logs to data/impedance_v2_*.csv. That prefix predates the rename of this file
and is left alone so the filenames still match my own archive.
"""
import asyncio, csv, datetime, math, sys, time

K    = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0    # N·m / rev
D    = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0     # N·m / (rev/s)
FF   = float(sys.argv[3]) if len(sys.argv) > 3 else 4.0      # N·m Coulomb comp
DUR  = float(sys.argv[4]) if len(sys.argv) > 4 else 60.0
KI   = float(sys.argv[5]) if len(sys.argv) > 5 else 300.0    # N·m/(rev.s) integrator
I_CLAMP = 10.0             # N·m: must clear breakaway even with a soft K
DITHER_NM = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0 # OFF by default: the
DITHER_HZ = 12.0           # 08-31 A/B showed dither adds vibration, buys nothing
DET_SCALE = 1.0            # exactly once, the friction map no longer carries it

# absolute-rotor-phase maps, fit 2026-08-31 (same as control/backdrive.py):
# TRUE friction map (2026-08-31): subtracting the detent field from each
# direction's raw profile yields IDENTICAL maps both ways, friction is
# symmetric; every direction difference was the conservative detent field.
# So: ONE friction map, and the detent term applied exactly ONCE at 1x.
# (The earlier per-direction maps + DET_SCALE 2 double/triple-counted the
# detent, assisting CCW, resisting CW: the up/down asymmetry was self-made.)
FRIC = [4.1364, 0.8021, 0.9766, 0.5987, -0.6532]
DET  = [0.0, -0.3213, 0.2221, 0.0955, 0.2534]       # conservative detent

def fric_mag(phase, om=0.0):
    return max(0.5, FRIC[0]
        + FRIC[1]*math.sin(2*math.pi*phase) + FRIC[2]*math.cos(2*math.pi*phase)
        + FRIC[3]*math.sin(4*math.pi*phase) + FRIC[4]*math.cos(4*math.pi*phase))

def det_tq(phase):
    return (DET[1]*math.sin(2*math.pi*phase) + DET[2]*math.cos(2*math.pi*phase)
            + DET[3]*math.sin(4*math.pi*phase) + DET[4]*math.cos(4*math.pi*phase))

MAX_TQ     = 12.0
GUARD_DEG  = 60.0          # abort if pushed this far from the hold point
W0         = 0.02          # rev/s: friction comp taper width (tanh)
PERIOD     = 0.001
WATCHDOG_S = 0.5
ABORT_FET  = 70.0
E1_SIGN    = -1.0          # AS5047P counts opposite to POSITION

LIMITS = {96,97,98,99,100,101,102,103}

async def main():
    import moteus
    qr = moteus.QueryResolution()
    qr.mode=moteus.INT8; qr.fault=moteus.INT8; qr.position=moteus.F32
    qr.velocity=moteus.F32; qr.torque=moteus.F32; qr.q_current=moteus.F32
    qr.voltage=moteus.F32; qr.temperature=moteus.INT8
    qr._extra = {moteus.Register.ENCODER_1_POSITION: moteus.F32}
    c = moteus.Controller(id=1, query_resolution=qr); R = moteus.Register

    await c.set_stop(); await asyncio.sleep(0.3)
    v0 = (await c.query()).values
    if int(v0[R.FAULT]) and int(v0[R.FAULT]) not in LIMITS:
        print(f"pre-existing fault {int(v0[R.FAULT])}"); return 1
    e1_home = v0[R.ENCODER_1_POSITION]
    p_home  = v0[R.POSITION]
    e1_prev = e1_home; th_unwrapped = 0.0
    print(f"K={K:.0f} N·m/rev  D={D:.0f}  FF={FF:.1f} N·m  {DUR:.0f} s")
    print("holding here, push the arm.\n")

    import os; os.makedirs("data", exist_ok=True)
    stamp = f"{datetime.datetime.now():%Y%m%d_%H%M%S}"
    fn = f"data/impedance_v2_{stamp}.csv"
    FIELDS = ["t_s","theta_out_deg","theta_orbis_deg","omega_rps","tau_cmd",
              "tau_meas","iq_A","fet_C","fault"]
    csvf = open(fn, "w", newline="")
    writer = csv.DictWriter(csvf, fieldnames=FIELDS); writer.writeheader()
    print(f"logging live to {fn}")
    rows = []; tau = 0.0; t0 = time.monotonic(); k = 0; last_print = 0.0
    integ = 0.0; t_prev = time.monotonic(); still_since = time.monotonic()
    attempt_done = False; attempt_t = time.monotonic()
    om_f = 0.0; anchor = 0.0
    try:
        while time.monotonic() - t0 < DUR:
            r = await c.set_position(position=math.nan, velocity=0.0,
                    kp_scale=0.0, kd_scale=0.0, ilimit_scale=0.0,
                    feedforward_torque=tau, maximum_torque=MAX_TQ,
                    watchdog_timeout=WATCHDOG_S, query=True)
            v = r.values; t = time.monotonic() - t0
            # unwrap the 1-turn absolute AS5047P into a continuous angle
            de = v[R.ENCODER_1_POSITION] - e1_prev
            if de > 0.5: de -= 1.0
            if de < -0.5: de += 1.0
            e1_prev = v[R.ENCODER_1_POSITION]
            th_unwrapped += de
            th = E1_SIGN * th_unwrapped          # output angle rel. start, POSITION sign
            om = v[R.VELOCITY]                   # Orbis-derived, clean
            # tanh taper instead of hard sign(omega): the comp fades smoothly
            # through zero velocity, so it cannot chatter against encoder noise
            now2 = time.monotonic(); dt = min(now2 - t_prev, 0.01); t_prev = now2
            err = 0.0 - th
            # SPRING PURITY RULE: while the arm is moving (hand or in flight),
            # torque is spring+damper+comp ONLY, integrator emptied so force
            # vs displacement has no history. Return machinery (integrator +
            # dither) engages only after the arm has been STILL for DWELL_S
            # with a standing error: a settled park.
            DWELL_S = 0.25
            BRK_NM  = 11.5                      # est. static breakaway, output N·m
            om_f += 0.08 * (om - om_f)      # ~11 Hz LP: balance of lag vs noise
            if abs(om) > 0.02:
                still_since = now2
                if abs(th - anchor) > 1.0/360: attempt_done = False  # real travel re-arms
            else:
                anchor = th
            was_parked = getattr(main, "_parked", False)
            parked_off = (now2 - still_since > DWELL_S) and abs(err)*360 > 0.5
            main._parked = parked_off
            if abs(om) > 0.05:
                integ *= (1 - 10.0*dt)          # hand moving: dump history fast
            elif err * integ < 0:
                integ = 0.0                     # crossed the target: dump
            elif parked_off:
                # ONE return attempt per settle: wind toward breakaway; if the
                # arm still hasn't moved once full authority is reached, a hand
                # must be holding it -> dump the integrator and stay a pure
                # spring until the next motion re-arms the attempt.
                avail = K*abs(err) + abs(integ)
                if not attempt_done:
                    if avail < BRK_NM + 0.5:
                        integ += KI * err * dt
                        attempt_t = now2
                    elif now2 - attempt_t > 0.3:
                        integ = 0.0             # attempt failed: hand assumed
                        attempt_done = True
            elif abs(err)*360 <= 0.3:
                integ *= (1 - 5.0*dt)           # home: bleed off
            integ = max(-I_CLAMP, min(I_CLAMP, integ))
            dith = 1.5 * math.sin(2*math.pi*DITHER_HZ*t) if parked_off else 0.0
            phase = (v[R.POSITION] * 50.0) % 1.0
            # phase-indexed friction comp (scaled by FF/4.14 to keep the FF arg
            # meaningful) + always-on detent cancellation + integrator
            # error-scheduled friction comp: full compensation while far from
            # target (ride the spring home without parking), easing back to the
            # ring-safe FF level inside ~6 deg
            ff_eff = FF + (4.14 - FF) * min(abs(err)*360/3.0, 1.0)
            tau = (K * err - D * om
                   + (ff_eff/4.14) * fric_mag(phase, om) * math.tanh(om_f / W0)
                   + DET_SCALE * det_tq(phase)
                   + integ + dith)
            tau = max(-MAX_TQ, min(MAX_TQ, tau))
            row = {"t_s": round(t,4), "theta_out_deg": round(th*360,4),
                   "theta_orbis_deg": round((v[R.POSITION]-p_home)*360,4),
                   "omega_rps": om, "tau_cmd": round(tau,3),
                   "tau_meas": v[R.TORQUE], "iq_A": v[R.Q_CURRENT],
                   "fet_C": v[R.TEMPERATURE], "fault": int(v[R.FAULT])}
            rows.append(row); writer.writerow(row)
            if k % 500 == 0: csvf.flush()
            f = int(v[R.FAULT])
            if f and f not in LIMITS: raise RuntimeError(f"fault {f}")
            if int(v[R.MODE]) == 1: raise RuntimeError("controller faulted")
            if int(v[R.MODE]) == 11: raise RuntimeError("watchdog mode 11")
            if abs(th*360) > GUARD_DEG: raise RuntimeError(f"guard: {th*360:+.0f} deg")
            if v[R.TEMPERATURE] > ABORT_FET: raise RuntimeError("fet hot")
            if t - last_print > 2.0:
                print(f"  t={t:5.1f}s  err {th*360:+7.2f} deg  tau {tau:+5.1f} N·m",
                      flush=True); last_print = t
            k += 1
            s = t0 + k*PERIOD - time.monotonic()
            if s > 0: await asyncio.sleep(s)
    except (Exception, KeyboardInterrupt) as e:
        print(f"\nSTOP: {e or type(e).__name__}")
    finally:
        try: await c.set_stop()
        except Exception: pass
        print("stopped. motor is off.")

    try: csvf.flush(); csvf.close()
    except Exception: pass
    print("->", fn, f"({len(rows)} samples)")
    return 0

sys.exit(asyncio.run(main()))
