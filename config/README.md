# Config

moteus configuration snapshots and the fitted friction map.

`friction_map.json` holds the friction and detent coefficients fitted from the
2026-08-30 and 08-31 sweeps, in the absolute rotor-phase frame.
`control/backdrive.py` carries the same numbers inline.

The three `.cfg` files are calibration states worth being able to get back to:
`after_recal10_20260824` is the working config after the recalibration that
fixed the 6.2 degree magnet rotation, `after_fw112_20260824` is after the
firmware update to 1.1.2, and `newboard_restored_20260828` is the replacement
board configured to match.

Restore one with `moteus_tool --restore-config <file>`, and recalibrate
afterwards if anything mechanical moved, especially the magnets.
