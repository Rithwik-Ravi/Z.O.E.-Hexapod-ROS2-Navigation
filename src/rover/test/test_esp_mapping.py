"""Unit tests for ESP32 radian-to-servo-degree mapping and safety clamping."""

import math
import pytest

from rover.esp import J1_NEUTRAL_DEG, positions_rad_to_servo_deg


def test_neutral_coxa_maps_to_90_degrees():
    """Calibrated coxa neutral angles must map to 90.0 deg on every leg."""
    positions_rad = [0.0] * 18
    for leg in range(1, 7):
        positions_rad[(leg - 1) * 3] = math.radians(J1_NEUTRAL_DEG[leg])

    deg = positions_rad_to_servo_deg(positions_rad)
    for leg in range(1, 7):
        assert deg[(leg - 1) * 3] == pytest.approx(90.0, abs=0.02)
        assert deg[(leg - 1) * 3 + 1] == pytest.approx(90.0, abs=0.02)
        assert deg[(leg - 1) * 3 + 2] == pytest.approx(90.0, abs=0.02)


def test_leg3_thigh_foot_inversion():
    """Leg 3 thigh and foot servos invert sign (`90 - deg`), unlike other legs."""
    positions_rad = [0.0] * 18
    # Set +10 deg on Leg 1 and Leg 3 thigh/foot
    delta = math.radians(10.0)
    positions_rad[1], positions_rad[2] = delta, delta
    positions_rad[7], positions_rad[8] = delta, delta

    deg = positions_rad_to_servo_deg(positions_rad)
    assert deg[1] == pytest.approx(100.0, abs=0.05)
    assert deg[2] == pytest.approx(100.0, abs=0.05)
    assert deg[7] == pytest.approx(80.0, abs=0.05)
    assert deg[8] == pytest.approx(80.0, abs=0.05)


def test_servo_limits_clamped_0_to_180():
    """Extreme joint angles must clamp strictly within [0.0, 180.0] deg."""
    deg = positions_rad_to_servo_deg([5.0, -5.0] * 9)
    assert all(0.0 <= d <= 180.0 for d in deg)


def test_invalid_joint_count_raises():
    """Non-18 joint arrays must raise ValueError."""
    with pytest.raises(ValueError):
        positions_rad_to_servo_deg([0.0] * 17)
