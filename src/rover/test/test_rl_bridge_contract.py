"""Unit tests verifying Z.O.E. Isaac Lab RL policy bridge observation & action contract."""

import numpy as np

from rover.rl_policy_bridge import (
    DEFAULT_JOINT_POS,
    JOINT_ORDER,
    MAX_SLEW_PER_STEP_RAD,
    NumpyMLPPolicy,
    apply_slew_rate_limit,
    build_actor_observation,
    is_cmd_stopped,
    project_gravity_from_quat,
)


def test_63d_observation_layout():
    """Verify exact 63-D observation dimensions and joint offset subtraction."""
    ang_vel = np.array([0.1, -0.2, 0.3], dtype=np.float32)
    proj_g = np.array([0.0, 0.0, -1.0], dtype=np.float32)
    cmd = np.array([0.2, 0.1, -0.3], dtype=np.float32)
    q = DEFAULT_JOINT_POS + 0.05
    dq = np.full(18, 0.25, dtype=np.float32)
    prev_a = np.full(18, -0.1, dtype=np.float32)

    obs = build_actor_observation(ang_vel, proj_g, cmd, q, dq, prev_a)
    assert obs.shape == (63,)
    assert len(JOINT_ORDER) == 18
    assert np.allclose(obs[9:27], 0.05, atol=1e-6)
    assert np.allclose(obs[27:45], 0.25, atol=1e-6)
    assert np.allclose(obs[45:63], -0.1, atol=1e-6)


def test_projected_gravity_identity_and_inverted():
    """Identity quaternion gives (0,0,-1); 180-deg roll gives (0,0,+1)."""
    g_upright = project_gravity_from_quat(0.0, 0.0, 0.0, 1.0)
    assert np.allclose(g_upright, [0.0, 0.0, -1.0], atol=1e-6)

    g_flipped = project_gravity_from_quat(1.0, 0.0, 0.0, 0.0)
    assert np.allclose(g_flipped, [0.0, 0.0, 1.0], atol=1e-6)


def test_slew_rate_limit_clamps_to_6_deg_per_step():
    """Per-step delta must clamp to +-6 deg (0.10472 rad)."""
    q_prev = np.zeros(18, dtype=np.float32)
    q_des = np.full(18, 1.0, dtype=np.float32)
    q_out = apply_slew_rate_limit(q_des, q_prev)
    assert np.allclose(q_out, MAX_SLEW_PER_STEP_RAD, atol=1e-6)


def test_is_cmd_stopped_deadband():
    """Pure yaw >= 0.05 rad/s must not be treated as stopped."""
    assert is_cmd_stopped(0.0, 0.0, 0.0) is True
    assert is_cmd_stopped(0.02, 0.02, 0.04) is True
    assert is_cmd_stopped(0.0, 0.0, 0.40) is False
    assert is_cmd_stopped(0.15, 0.0, 0.0) is False


def test_numpy_mlp_policy_forward(tmp_path):
    """Verify NumpyMLPPolicy loads exported npz weights and outputs bounded [-1, 1] actions."""
    npz_file = tmp_path / 'policy_mlp.npz'
    np.savez(
        npz_file,
        obs_mean=np.zeros(63, dtype=np.float32),
        obs_std=np.ones(63, dtype=np.float32),
        w0=np.zeros((32, 63), dtype=np.float32),
        b0=np.zeros(32, dtype=np.float32),
        w1=np.zeros((18, 32), dtype=np.float32),
        b1=np.full(18, 0.25, dtype=np.float32),
    )
    policy = NumpyMLPPolicy(str(npz_file))
    out = policy(np.zeros(63, dtype=np.float32))
    assert out.shape == (18,)
    assert np.allclose(out, 0.25, atol=1e-6)
