"""Abort limits and the check that enforces them.

Every script calls check_limits() on every sample. If it raises, the caller
stops the motor. The limits are deliberately well inside the hardware ratings:
this is a one-off build with a $100 gearbox in it, and I would rather abort a
good run than cook a winding.
"""
import math

from .registers import MOTEUS_FAULTS, MOTEUS_LIMIT_REASONS

ABORT_FET_TEMP_C = 70.0    # moteus derates ~100 C; large margin for a no-load test
ABORT_Q_CURRENT_A = 6.0    # servo.max_current_A is 7.5; trip below it
ABORT_BUS_V_LOW = 20.0     # 24 V supply sagging = PSU hit its current limit
ABORT_BUS_V_HIGH = 30.0    # regen on decel pumping the rail

# Three things that look like faults and are not, so nothing here trips on them:
#   motor_temp_C  no thermistor is fitted, it reads a constant 0. fet_temp_C is
#                 the only real thermal measurement on this rig.
#   aux1.spi      the AS5047P output encoder. Its own reading is not a safety
#                 signal, and on the motor-only runs there was no magnet at all.
#   crc_errors    a climbing count is a finding to write up, not a reason to
#                 stop. Commutation runs off the Orbis, and if that link really
#                 fails the controller raises an encoder fault by itself.


class Abort(Exception):
    pass


def check_limits(s):
    """Raise Abort if any sample violates a safety limit."""
    f = s["fault"]
    if f == -1:
        raise Abort("FAULT register absent from reply (query truncated?), "
                    "the primary safety check is blind, refusing to continue")
    if f in MOTEUS_LIMIT_REASONS:
        pass  # status, not a fault: logged via CSV, not aborted on
    elif f and f > 0:
        raise Abort(f"controller fault {f} ({MOTEUS_FAULTS.get(f, 'unknown')})")

    if s["mode"] == 1:
        raise Abort("controller entered fault mode")
    if s["mode"] == 11:
        raise Abort("controller in TIMEOUT mode (11): command watchdog tripped, "
                    "it ignores position commands until set_stop")

    t = s["fet_temp_C"]
    if not math.isnan(t) and t > ABORT_FET_TEMP_C:
        raise Abort(f"fet_temp_C {t:.1f} > {ABORT_FET_TEMP_C} C")

    q = s["q_current_A"]
    if not math.isnan(q) and abs(q) > ABORT_Q_CURRENT_A:
        raise Abort(f"q_current {q:.2f} A exceeds {ABORT_Q_CURRENT_A} A trip")

    bv = s["bus_voltage_V"]
    if not math.isnan(bv):
        if bv < ABORT_BUS_V_LOW:
            raise Abort(f"bus voltage {bv:.1f} V sagging below {ABORT_BUS_V_LOW} V "
                        f"(PSU current limit?)")
        if bv > ABORT_BUS_V_HIGH:
            raise Abort(f"bus voltage {bv:.1f} V above {ABORT_BUS_V_HIGH} V (regen)")
