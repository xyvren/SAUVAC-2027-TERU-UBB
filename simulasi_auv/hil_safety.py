import math


HIL_MAX_DELTA_US = 75
HIL_WATCHDOG_MS = 350


def validate_hil_pulses(pulses):
    if not isinstance(pulses, list) or len(pulses) != 6:
        raise ValueError("pulses must contain exactly six values")

    validated = []
    low = 1500 - HIL_MAX_DELTA_US
    high = 1500 + HIL_MAX_DELTA_US
    for pulse in pulses:
        if isinstance(pulse, bool) or not isinstance(pulse, (int, float)):
            raise ValueError("HIL pulses must be integer microseconds")
        if isinstance(pulse, float) and (not math.isfinite(pulse) or not pulse.is_integer()):
            raise ValueError("HIL pulses must be finite integer microseconds")
        pulse = int(pulse)
        if not low <= pulse <= high:
            raise ValueError(f"HIL pulses must be between {low} and {high} microseconds")
        validated.append(pulse)
    return validated


def validate_hil_sensors(sensors):
    if not isinstance(sensors, dict):
        raise ValueError("sensors must be an object")

    depth = sensors.get("depth_cm")
    pitch = sensors.get("pitch")
    roll = sensors.get("roll")
    gyro = sensors.get("gyro_dps")
    values = [depth, pitch, roll]
    if not isinstance(gyro, list) or len(gyro) != 3:
        raise ValueError("gyro_dps must contain exactly three values")
    values.extend(gyro)
    if any(isinstance(value, bool) or not isinstance(value, (int, float)) for value in values):
        raise ValueError("HIL sensor values must be numeric")
    if any(isinstance(value, float) and not math.isfinite(value) for value in values):
        raise ValueError("HIL sensor values must be finite")
    if not 0.0 <= depth <= 140.0:
        raise ValueError("depth_cm must be between 0 and 140")
    if not -90.0 <= pitch <= 90.0 or not -90.0 <= roll <= 90.0:
        raise ValueError("pitch and roll must be between -90 and 90 degrees")
    if any(abs(rate) > 2000.0 for rate in gyro):
        raise ValueError("gyro rates must be between -2000 and 2000 degrees per second")
    return {
        "depth_cm": float(depth),
        "pitch": float(pitch),
        "roll": float(roll),
        "gyro_dps": [float(rate) for rate in gyro],
    }
