# Data

The runs behind the figures and the numbers in the writeup. Curated, not the
full log directory. Files are `<test>_<date>_<time>.csv`, one row per sample,
written by the script of the same name. Anything that could not be read is
`nan` rather than dropped, so gaps stay visible in a plot instead of quietly
closing up.

The impedance runs need care because they are not interchangeable. `170104` is
the one that spans the ringing onset, clean at 12,800, first ringing at 25,600
and clearly ringing at 51,200, and it feeds both panels of the figure. `170011`
covers the low end and supplies the extra overshoot points. `170155` is the
same ladder with Coulomb feedforward on, which changes steady-state error, so
it belongs to a different question and is not mixed in. `165730` is kept for
the record but should not be plotted: `kp_scale=0` does not scale `ki` or
`ilimit`, so the integrator kept winding on a run where it should have been
inert, and the friction its steady-state error implies is nonsense.

The friction sweeps are the source of `config/friction_map.json` and show the
once-per-rev eccentricity hump with the twice-per-rev ellipse ripple on top.
`gain_sweep` has the stall fractions that picked kp 8000. `breakaway_ramp` is
where roughly 10 N·m comes from. The `lever_*_steps` files are commanded torque
against what a scale actually read on a lever arm: slope is transmission
efficiency, x-intercept is static friction.

Two PNGs explain why efficiency is measured the way it is.
`lever_stall_20260830.png` is the stall staircase, where commanding 8, 9 and 10
N·m against a blocked scale returns nearly the same reading because stiction
locks the output between slip events. `gravity_dyno_20260831.png` is the method
that replaced it, sweeping a weighted mass up and down so friction adds one way,
subtracts the other, and cancels.

The rest: `magnet_check_washer_*` is the Orbis disturbance from the output
magnet with the back-iron washer off, glued, and at the 5 degree position.
`backemf_20260816` is one motor-only run that produced two numbers, the CAN-FD
loop period (0.91 ms mean, 1.84 ms p99, the T in the passivity bound) and an
independent Kt estimate. `interference_20260824` is the encoder crosstalk
measurement, and the `motor_sweep` PNGs are the no-load baseline from before
the gearbox went on.

`impedance_v2_*` and `transparent_v2_*` are runs from `control/impedance.py`
and `control/backdrive.py`. Those two files were renamed when this repo was
organized and the log prefixes were not, so the filenames still match my own
archive.
