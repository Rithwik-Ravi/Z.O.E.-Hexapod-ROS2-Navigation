"""Unit tests for hexapod analytical 3-DOF IK and tripod gait logic."""

import math

from rover.hxp_command import COXA_LENGTH_MM, FEMUR_LENGTH_MM, HEXAPOD_JOINT_NAMES, TIBIA_LENGTH_MM, solve_leg_ik


def test_joint_order_has_18_leg_grouped_joints():
    """Verify canonical 18-DOF leg-grouped joint naming."""
    assert len(HEXAPOD_JOINT_NAMES) == 18
    for leg in range(1, 7):
        base = (leg - 1) * 3
        assert HEXAPOD_JOINT_NAMES[base] == f'NCCUP{leg}_joint'
        assert HEXAPOD_JOINT_NAMES[base + 1] == f'NTHIGH{leg}_joint'
        assert HEXAPOD_JOINT_NAMES[base + 2] == f'NFOOT{leg}_joint'


def test_ik_forward_kinematics_consistency():
    """Verify solve_leg_ik produces finite angles within physical servo limits for stance poses."""
    stance_points = [
        (77.8, 77.8, 150.0),
        (55.0, 80.0, 150.0),
        (80.0, 55.0, 150.0),
        (77.8, 77.8, 100.0),
    ]
    for x, y, z in stance_points:
        j1, j2, j3 = solve_leg_ik(x, y, z)
        assert all(math.isfinite(a) for a in (j1, j2, j3))
        assert -1.5 <= j1 <= 1.5
        assert -1.5 <= j2 <= 1.5
        assert -1.5 <= j3 <= 1.5
        assert math.isclose(j1, math.atan2(x, y), rel_tol=1e-6)


def test_ik_reconstructs_foot_distance():
    """Verify triangle geometry in the sagittal plane matches target reach."""
    x, y, z = 60.0, 80.0, 120.0
    j1, j2, j3 = solve_leg_ik(x, y, z)
    horiz = math.hypot(x, y) - COXA_LENGTH_MM
    target_dist = math.hypot(z, horiz)
    # Interior knee angle is 1.57 - j3 (from solve_leg_ik definition)
    knee = 1.57 - j3
    reconstructed = math.sqrt(
        FEMUR_LENGTH_MM ** 2 + TIBIA_LENGTH_MM ** 2
        - 2.0 * FEMUR_LENGTH_MM * TIBIA_LENGTH_MM * math.cos(knee)
    )
    assert math.isclose(reconstructed, target_dist, rel_tol=1e-5)
