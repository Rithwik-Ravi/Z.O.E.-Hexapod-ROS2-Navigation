"""Unit tests for odom_from_tf body-frame velocity transformation and math."""

import math
from types import SimpleNamespace
import pytest

from rover.odom_from_tf import normalize_angle, world_vel_to_body, yaw_from_quaternion


def test_world_vel_to_body_cardinal_yaws():
    """Verify world-frame velocities rotate accurately into base_footprint frame."""
    # Facing East (yaw = 0): +X world -> +X body
    vx, vy = world_vel_to_body(0.3, 0.0, 0.0)
    assert vx == pytest.approx(0.3, abs=1e-6)
    assert vy == pytest.approx(0.0, abs=1e-6)

    # Facing North (yaw = +pi/2): +Y world -> +X body, +X world -> -Y body
    vx, vy = world_vel_to_body(0.0, 0.25, math.pi / 2.0)
    assert vx == pytest.approx(0.25, abs=1e-6)
    assert vy == pytest.approx(0.0, abs=1e-6)

    # Facing West (yaw = pi): -X world -> +X body
    vx, vy = world_vel_to_body(-0.2, 0.1, math.pi)
    assert vx == pytest.approx(0.2, abs=1e-6)
    assert vy == pytest.approx(-0.1, abs=1e-6)


def test_yaw_from_quaternion():
    """Verify planar yaw extraction for 0, +90, and -90 degrees."""
    for target_deg in [0.0, 45.0, 90.0, -90.0, 179.0]:
        rad = math.radians(target_deg)
        q = SimpleNamespace(x=0.0, y=0.0, z=math.sin(rad / 2.0), w=math.cos(rad / 2.0))
        assert yaw_from_quaternion(q) == pytest.approx(rad, abs=1e-6)


def test_normalize_angle():
    """Verify angle wrapping into [-pi, pi]."""
    assert normalize_angle(3.5 * math.pi) == pytest.approx(-0.5 * math.pi, abs=1e-6)
    assert normalize_angle(-3.5 * math.pi) == pytest.approx(0.5 * math.pi, abs=1e-6)
