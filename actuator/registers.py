"""Moteus register decoding: mode/fault tables and unpack().

Everything that turns a raw moteus reply into numbers I can log. It lives here
because six different test scripts need the same decode and the same fault
names, and I got tired of them drifting apart.
"""
import math

MOTEUS_MODES = {
    0: "stopped", 1: "fault", 2: "enabling", 3: "calibrating",
    4: "calibration complete", 5: "pwm", 6: "voltage", 7: "voltage_foc",
    8: "voltage_dq", 9: "current", 10: "position", 11: "timeout",
    12: "zero_velocity", 13: "stay_within", 14: "measure_ind",
    15: "brake",
}

MOTEUS_FAULTS = {
    32: "calibration fault", 33: "motor driver fault", 34: "over voltage",
    35: "encoder fault", 36: "motor not configured", 37: "pwm cycle overrun",
    38: "over temperature", 39: "outside limit",
    40: "under voltage", 41: "config changed", 42: "THETA INVALID "
        "(no valid commutation angle: calibration incomplete/invalid)",
    43: "position invalid", 44: "driver enable fault",
    45: "stop position deprecated", 46: "timing violation",
    47: "bemf feedforward no accel", 48: "invalid limits",
    49: "position control error", 50: "velocity control error",
}

# 96-103 are NOT hardware faults. They're reported through the SAME fault
# register (0x00f) but mean "this is what's currently limiting the output",
# a normal status during valid operation. Added in a 2025-08-07 firmware
# feature ("output limit reason reporting"). Must NOT be treated as an abort
# condition, e.g. code 102 just means "your own commanded max_torque is
# what's capping the output right now", which is the limiter working
# correctly, not a failure.
MOTEUS_LIMIT_REASONS = {
    96: "servo.max_velocity", 97: "servo.max_power_W",
    98: "maximum system voltage", 99: "servo.max_current_A",
    100: "servo.fault_temperature", 101: "servo.motor_fault_temperature",
    102: "commanded maximum torque", 103: "position bounds "
         "(servopos.position_min/max)",
}



def unpack(moteus, result, slow=None):
    """moteus Result into plain dict. Missing registers come back as nan.

    `slow` carries the most recent MOTOR_TEMPERATURE / ENCODER_VALIDITY values,
    which are polled separately to keep the fast reply inside one CAN-FD frame.
    """
    R = moteus.Register
    v = result.values
    slow = slow or {}

    def g(reg):
        return v.get(reg, float("nan"))

    voltage = g(R.VOLTAGE)
    power = g(R.POWER)

    # No bus-current register exists in the moteus telemetry map.
    # Derive it from reported electrical power and bus voltage.
    if voltage and not math.isnan(voltage) and not math.isnan(power) and abs(voltage) > 1.0:
        bus_current = power / voltage
    else:
        bus_current = float("nan")

    return {
        "mode": int(g(R.MODE)) if not math.isnan(g(R.MODE)) else -1,
        "position_rev_out": g(R.POSITION),
        "velocity_rps_out": g(R.VELOCITY),
        "torque_Nm_out": g(R.TORQUE),
        "q_current_A": g(R.Q_CURRENT),
        "d_current_A": g(R.D_CURRENT),
        "bus_voltage_V": voltage,
        "power_W": power,
        "bus_current_A_derived": bus_current,
        "fet_temp_C": g(R.TEMPERATURE),
        "motor_temp_C": slow.get("motor_temp_C", float("nan")),
        "fault": int(g(R.FAULT)) if not math.isnan(g(R.FAULT)) else -1,
        # Orbis via the fast register path (output-referred, scaled by 0.02)
        "orbis_position_rev_out": g(R.ENCODER_0_POSITION),
        "orbis_velocity_rps_out": g(R.ENCODER_0_VELOCITY),
        "encoder_validity": slow.get("encoder_validity", float("nan")),
    }
