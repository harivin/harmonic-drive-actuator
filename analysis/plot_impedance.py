#!/usr/bin/env python3
"""The impedance-control figure for the writeup.

Two panels

  A) Step response family across K (3200 -> 51200), from data/impedance_20260821_170104.csv.
     You can watch the response go from critically clean to visibly ringing.

  B) Overshoot vs K on a log axis, with BOTH the measured ringing onset and the
     Colgate-Hogan passivity bound marked. The gap between them is the finding:
     the actuator went unstable at ~10% of the stiffness the sample rate allowed,
     because the flexspline's own torsional resonance binds first.

Four impedance runs are not interchangeable:
  165730  EXCLUDED. K*ss_err implies 68 and 140 N·m of friction at high K, which is
          the stale-integrator bug (kp_scale=0 does not scale ki/ilimit). Bogus run.
  170011  FF_FRICTION = 0, K = 50..3200. Clean. Used for the low-K overshoot points.
  170104  K = 3200..51200. The only run spanning the ringing onset. Panel A + B.
  170155  FF_FRICTION = 6 (Coulomb comp). Not used here, it changes steady-state
          error, which is a different figure.

Overshoot is 0.0 for every clean run at K <= 3200, so combining 170011 with 170104
for panel B is safe; steady-state error would NOT be safe to combine that way.

Usage:  python3 analysis/plot_impedance.py
Writes: data/fig_impedance_step_response.png
"""
import csv
import glob
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

RUN_FAMILY = "data/impedance_20260821_170104.csv"
SUMMARIES_KEEP = ["data/impedance_20260821_170011_summary.csv",
                  "data/impedance_20260821_170104_summary.csv"]

STEP_DEG = 18.0          # STEP_REV 0.05 rev, output-referred
T_P99_S = 0.00184        # loop period p99, measured 2026-08-16 (data/backemf_*.csv)

# K goes light to dark with stiffness so the traces read in order. Dropped
# K = 6400: at 0.13% overshoot it sits on top of 3200 and just crowds the plot.
K_RAMP = {
    3200:  "#4292c6",
    12800: "#2171b5",
    25600: "#08519c",
    51200: "#08306b",
}
INK        = "#1a1a1a"
INK_MUTED  = "#6b6b6b"
GRID       = "#dddddd"
ACCENT     = "#0072B2"   # same blue as the other plots in the repo
FLAG       = "#D55E00"   # only used to mark the ringing onset


def load_family(path):
    """Return {K: [(t, deg_from_home), ...]} for the step-response panel."""
    if not os.path.exists(path):
        sys.exit(f"missing {path}")
    by_k = {}
    for r in csv.DictReader(open(path)):
        K = int(float(r["K"]))
        by_k.setdefault(K, []).append(
            (float(r["t_s"]), float(r["theta"]), float(r["theta_d"])))
    out = {}
    for K, rows in by_k.items():
        rows.sort(key=lambda x: x[0])
        # theta_d before the step IS home; express travel relative to it.
        pre = [th_d for t, th, th_d in rows if t < 0]
        home = pre[0] if pre else rows[0][2]
        out[K] = [(t, (th - home) * 360.0) for t, th, th_d in rows]
    return out


def load_overshoot(paths):
    """Return sorted [(K, overshoot_pct, K_over_bound)] from the kept summaries."""
    seen = {}
    for p in paths:
        if not os.path.exists(p):
            continue
        for r in csv.DictReader(open(p)):
            K = int(float(r["K"]))
            ov = float(r["overshoot_pct"])
            kob = float(r["K_over_bound"])
            # If a K appears in both files, keep the larger overshoot, the
            # conservative reading, and it only matters at the K=3200 overlap
            # where both are 0.0 anyway.
            if K not in seen or ov > seen[K][0]:
                seen[K] = (ov, kob)
    return sorted((K, v[0], v[1]) for K, v in seen.items())


def main():
    family = load_family(RUN_FAMILY)
    over = load_overshoot(SUMMARIES_KEEP)
    if not family or not over:
        sys.exit("no data loaded")

    # The passivity bound at the onset K, back-computed from the logged ratio.
    onset_K = next((K for K, ov, _ in over if ov > 5.0), None)
    bound_K = None
    for K, ov, kob in over:
        if K == onset_K and kob:
            bound_K = K / kob
    ratio = (onset_K / bound_K) if (onset_K and bound_K) else None

    fig, (axA, axB) = plt.subplots(1, 2, figsize=(12.5, 4.9))

    # ---------------------------------------------------------------- panel A
    T_LO, T_HI = -0.08, 0.62
    peaks = {}
    for K in sorted(K_RAMP):
        if K not in family:
            continue
        seg = [(t, d) for t, d in family[K] if T_LO <= t <= T_HI]
        axA.plot([t for t, _ in seg], [d for _, d in seg],
                 color=K_RAMP[K], lw=1.7,
                 label=f"K = {K:,}", zorder=3 if K < 25600 else 4)
        # Remember each trace's peak so the ringing ones can be labeled where
        # they are actually separated. Labels at the right-hand end would all
        # collide: every trace settles to the same 18 deg setpoint.
        post = [(t, d) for t, d in seg if t > 0]
        if post:
            peaks[K] = max(post, key=lambda x: x[1])

    axA.axhline(STEP_DEG, color=INK_MUTED, ls="--", lw=1, zorder=1)
    axA.annotate(f"{STEP_DEG:.0f}° setpoint", xy=(T_HI - 0.01, STEP_DEG),
                 xytext=(0, -12), textcoords="offset points",
                 ha="right", va="top", fontsize=8.5, color=INK_MUTED)
    axA.axvline(0, color=GRID, lw=1, zorder=1)

    # ONE selective callout, on the extreme trace only, with a short straight
    # leader in muted ink. Two callouts needed long curved leaders that read as
    # extra data traces, and the peaks are only 0.5 deg apart (19.7 vs 19.2) so
    # they cannot be labeled separately without colliding. Panel B carries the
    # per-K overshoot numbers; repeating them here would be a number on every
    # point for no gain.
    ov_by_K = {K: ov for K, ov, _ in over}
    if 51200 in peaks:
        px, py = peaks[51200]
        axA.annotate(f"K = 51,200 peaks {py:.1f}°\n"
                     f"({ov_by_K.get(51200, 0):.1f}% overshoot)",
                     xy=(px, py), xytext=(0.245, 20.6), textcoords="data",
                     fontsize=8.5, color=INK, ha="left", va="center",
                     arrowprops=dict(arrowstyle="-", color=INK_MUTED,
                                     lw=0.9, shrinkA=2, shrinkB=4))
    axA.set_ylim(-1.6, 22.4)

    axA.set_xlabel("time from step (s)", fontsize=9.5, color=INK)
    axA.set_ylabel("output position (deg)", fontsize=9.5, color=INK)
    axA.set_title("Step response stiffens, then starts to ring",
                  fontsize=11, color=INK, loc="left", pad=9)
    axA.set_xlim(T_LO, T_HI)
    axA.grid(True, color=GRID, lw=0.7)
    axA.set_axisbelow(True)
    leg = axA.legend(loc="lower right", fontsize=8.5, frameon=True, ncol=2,
                     framealpha=0.95, edgecolor=GRID, title="stiffness (N·m/rev)")
    leg.get_title().set_fontsize(8.5)
    leg.get_title().set_color(INK_MUTED)
    for t in leg.get_texts():
        t.set_color(INK)

    # ---------------------------------------------------------------- panel B
    Ks = [K for K, _, _ in over]
    ovs = [ov for _, ov, _ in over]
    axB.plot(Ks, ovs, color=ACCENT, lw=2, marker="o", ms=6,
             mfc="white", mec=ACCENT, mew=1.8, zorder=3)

    if onset_K is not None:
        oy = dict(zip(Ks, ovs))[onset_K]
        axB.plot([onset_K], [oy], marker="o", ms=9, color=FLAG, zorder=5)
        axB.annotate(f"ringing onset\nK = {onset_K:,}   {oy:.1f}% overshoot",
                     xy=(onset_K, oy), xytext=(-26, 30),
                     textcoords="offset points", ha="right", va="center",
                     fontsize=9, color=FLAG,
                     arrowprops=dict(arrowstyle="-", color=FLAG, lw=1.2,
                                     shrinkA=2, shrinkB=5))

    if bound_K is not None:
        axB.axvline(bound_K, color=INK_MUTED, ls="--", lw=1.4, zorder=2)
        # Rotated and hugging the rule: horizontal text here ran straight across
        # the data and into the onset callout.
        axB.annotate(f"Colgate–Hogan passivity bound   K ≈ {bound_K:,.0f}",
                     xy=(bound_K, max(ovs) * 0.52), xytext=(-7, 0),
                     textcoords="offset points", rotation=90,
                     ha="right", va="center", fontsize=8.5, color=INK_MUTED)

    axB.set_xscale("log")
    axB.set_xlim(min(Ks) * 0.6, (bound_K or max(Ks)) * 2.2)
    axB.set_ylim(-0.6, max(ovs) * 1.32)
    axB.set_xlabel("commanded stiffness K (N·m/rev)", fontsize=9.5, color=INK)
    axB.set_ylabel("overshoot (%)", fontsize=9.5, color=INK)
    axB.set_title("Instability arrives ~10× before the sample rate says it should",
                  fontsize=11, color=INK, loc="left", pad=9)
    axB.grid(True, color=GRID, lw=0.7, which="major")
    axB.set_axisbelow(True)

    if ratio:
        axB.text(0.015, 0.94,
                 f"onset / bound = {ratio:.2f}\nthe flexspline resonance binds first",
                 transform=axB.transAxes, fontsize=9, va="top", color=INK,
                 bbox=dict(boxstyle="round,pad=0.42", fc="white",
                           ec=GRID, lw=1))

    for ax in (axA, axB):
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
        for s in ("bottom", "left"):
            ax.spines[s].set_color(GRID)
        ax.tick_params(colors=INK_MUTED, labelsize=8.5)

    fig.tight_layout(rect=(0, 0.062, 1, 1))
    fig.text(0.008, 0.022,
             "Impedance control on a 50:1 harmonic drive: 18 deg step, zeta = 0.5, "
             "D = 2ζ√(KJ), 15 N·m clamp, τ = K(θd−θ) − Dω computed in Python "
             "at 1.5 ms over CAN-FD.  Source: data/impedance_20260821_170104.csv",
             fontsize=8, color=INK_MUTED, ha="left", va="bottom")
    out = "data/fig_impedance_step_response.png"
    fig.savefig(out, dpi=170, facecolor="white")
    print(f"wrote {out}")

    print(f"\n  measured ringing onset : K = {onset_K:,}")
    print(f"  passivity bound        : K = {bound_K:,.0f}")
    print(f"  ratio                  : {ratio:.3f}")
    print("\n  overshoot vs K:")
    for K, ov, _ in over:
        print(f"    K = {K:>6,}   {ov:6.2f} %")


main()
