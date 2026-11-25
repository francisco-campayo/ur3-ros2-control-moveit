"""Unit tests for trajectory_utils module.

These tests are pure Python with no ROS2 dependencies, verifying the
trapezoidal velocity profile generation in isolation.
"""

import pytest

from my_ur_control.trajectory_utils import (
    TrajectoryPoint,
    generate_trapezoidal_trajectory,
)


class TestTrapezoidalTrajectory:
    """Tests for trapezoidal trajectory generation with per-joint limits."""

    def test_simple_trajectory_reaches_target(self):
        """Test that a simple trajectory reaches the target position."""
        q_start = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        q_end = [1.0, -1.0, 0.5, -0.5, 0.25, -0.25]
        v_max = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        a_max = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
        dt = 0.01

        trajectory = generate_trapezoidal_trajectory(q_start, q_end, v_max, a_max, dt)

        assert len(trajectory) > 0

        final_point = trajectory[-1]
        for i in range(len(q_end)):
            assert abs(final_point.positions[i] - q_end[i]) < 1e-6

        for v in final_point.velocities:
            assert abs(v) < 1e-6

    def test_per_joint_limits(self):
        """Test that per-joint limits are respected."""
        q_start = [0.0] * 6
        q_end = [2.0] * 6
        v_max = [1.0, 1.0, 1.0, 2.0, 2.0, 2.0]  # Different limits per joint
        a_max = [2.0, 2.0, 2.0, 4.0, 4.0, 4.0]
        dt = 0.01

        trajectory = generate_trapezoidal_trajectory(q_start, q_end, v_max, a_max, dt)

        for point in trajectory:
            for i in range(6):
                assert abs(point.velocities[i]) <= v_max[i] + 1e-6

    def test_zero_distance_returns_single_point(self):
        """Test that zero distance returns a single point at start."""
        q_start = [0.5, -0.5, 0.0, 1.0, -1.0, 0.0]
        q_end = [0.5, -0.5, 0.0, 1.0, -1.0, 0.0]
        v_max = [1.0] * 6
        a_max = [2.0] * 6
        dt = 0.01

        trajectory = generate_trapezoidal_trajectory(q_start, q_end, v_max, a_max, dt)

        assert len(trajectory) == 1
        assert trajectory[0].positions == q_start
        assert trajectory[0].velocities == [0.0] * 6
        assert trajectory[0].time_from_start == 0.0

    def test_velocity_limit_respected(self):
        """Test that velocity never exceeds v_max for any joint."""
        q_start = [0.0] * 6
        q_end = [2.0] * 6
        v_max = [1.0, 1.0, 1.0, 2.0, 2.0, 2.0]
        a_max = [2.0, 2.0, 2.0, 4.0, 4.0, 4.0]
        dt = 0.01

        trajectory = generate_trapezoidal_trajectory(q_start, q_end, v_max, a_max, dt)

        for point in trajectory:
            for i in range(6):
                assert abs(point.velocities[i]) <= v_max[i] + 1e-6

    def test_acceleration_limit_respected(self):
        """Test that acceleration never exceeds a_max for any joint."""
        q_start = [0.0] * 6
        q_end = [2.0] * 6
        v_max = [1.0, 1.0, 1.0, 2.0, 2.0, 2.0]
        a_max = [2.0, 2.0, 2.0, 4.0, 4.0, 4.0]
        dt = 0.01

        trajectory = generate_trapezoidal_trajectory(q_start, q_end, v_max, a_max, dt)

        for i in range(1, len(trajectory)):
            t_diff = trajectory[i].time_from_start - trajectory[i - 1].time_from_start
            if t_diff == 0:
                continue
            for j in range(6):
                vel_diff = trajectory[i].velocities[j] - trajectory[i - 1].velocities[j]
                accel = abs(vel_diff / t_diff)
                assert accel <= a_max[j] + 1e-6

    def test_monotonic_time(self):
        """Test that time increases monotonically."""
        q_start = [0.0] * 6
        q_end = [1.0] * 6
        v_max = [1.0] * 6
        a_max = [2.0] * 6
        dt = 0.01

        trajectory = generate_trapezoidal_trajectory(q_start, q_end, v_max, a_max, dt)

        for i in range(1, len(trajectory)):
            assert trajectory[i].time_from_start >= trajectory[i - 1].time_from_start

    def test_start_position_correct(self):
        """Test that trajectory starts at q_start."""
        q_start = [0.5, -0.5, 0.0, 1.0, -1.0, 0.0]
        q_end = [1.0, -1.0, 0.5, -0.5, 0.25, -0.25]
        v_max = [1.0] * 6
        a_max = [2.0] * 6
        dt = 0.01

        trajectory = generate_trapezoidal_trajectory(q_start, q_end, v_max, a_max, dt)

        first_point = trajectory[0]
        for i in range(len(q_start)):
            assert abs(first_point.positions[i] - q_start[i]) < 1e-6

    def test_negative_displacement(self):
        """Test trajectory with negative displacement."""
        q_start = [1.0, -1.0, 0.5, -0.5, 0.25, -0.25]
        q_end = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        v_max = [1.0] * 6
        a_max = [2.0] * 6
        dt = 0.01

        trajectory = generate_trapezoidal_trajectory(q_start, q_end, v_max, a_max, dt)

        final_point = trajectory[-1]
        for i in range(len(q_end)):
            assert abs(final_point.positions[i] - q_end[i]) < 1e-6

    def test_triangular_profile_short_distance(self):
        """Test that short distances use triangular profile."""
        q_start = [0.0] * 6
        q_end = [0.1] * 6
        v_max = [10.0] * 6
        a_max = [2.0] * 6
        dt = 0.01

        trajectory = generate_trapezoidal_trajectory(q_start, q_end, v_max, a_max, dt)

        max_vel = max(abs(v) for point in trajectory for v in point.velocities)
        assert max_vel < max(v_max)

    def test_trapezoidal_profile_long_distance(self):
        """Test that long distances use trapezoidal profile."""
        q_start = [0.0] * 6
        q_end = [5.0] * 6
        v_max = [1.0] * 6
        a_max = [2.0] * 6
        dt = 0.01

        trajectory = generate_trapezoidal_trajectory(q_start, q_end, v_max, a_max, dt)

        max_vel = max(abs(v) for point in trajectory for v in point.velocities)
        assert abs(max_vel - max(v_max)) < 1e-6


class TestInputValidation:
    """Tests for input validation."""

    def test_mismatched_lengths_raises_error(self):
        """Test that mismatched q_start and q_end lengths raise ValueError."""
        q_start = [0.0] * 6
        q_end = [1.0] * 5
        v_max = [1.0] * 6
        a_max = [2.0] * 6

        with pytest.raises(ValueError, match="same length"):
            generate_trapezoidal_trajectory(q_start, q_end, v_max, a_max, 0.01)

    def test_mismatched_v_max_length_raises_error(self):
        """Test that mismatched v_max length raises ValueError."""
        q_start = [0.0] * 6
        q_end = [1.0] * 6
        v_max = [1.0] * 5
        a_max = [2.0] * 6

        with pytest.raises(ValueError, match="v_max must have the same length"):
            generate_trapezoidal_trajectory(q_start, q_end, v_max, a_max, 0.01)

    def test_mismatched_a_max_length_raises_error(self):
        """Test that mismatched a_max length raises ValueError."""
        q_start = [0.0] * 6
        q_end = [1.0] * 6
        v_max = [1.0] * 6
        a_max = [2.0] * 5

        with pytest.raises(ValueError, match="a_max must have the same length"):
            generate_trapezoidal_trajectory(q_start, q_end, v_max, a_max, 0.01)

    def test_negative_v_max_raises_error(self):
        """Test that negative v_max raises ValueError."""
        q_start = [0.0] * 6
        q_end = [1.0] * 6
        v_max = [-1.0] * 6
        a_max = [2.0] * 6

        with pytest.raises(ValueError, match="All v_max values must be finite and positive"):
            generate_trapezoidal_trajectory(q_start, q_end, v_max, a_max, 0.01)

    def test_zero_v_max_raises_error(self):
        """Test that zero v_max raises ValueError."""
        q_start = [0.0] * 6
        q_end = [1.0] * 6
        v_max = [0.0] * 6
        a_max = [2.0] * 6

        with pytest.raises(ValueError, match="All v_max values must be finite and positive"):
            generate_trapezoidal_trajectory(q_start, q_end, v_max, a_max, 0.01)

    def test_negative_a_max_raises_error(self):
        """Test that negative a_max raises ValueError."""
        q_start = [0.0] * 6
        q_end = [1.0] * 6
        v_max = [1.0] * 6
        a_max = [-2.0] * 6

        with pytest.raises(ValueError, match="All a_max values must be finite and positive"):
            generate_trapezoidal_trajectory(q_start, q_end, v_max, a_max, 0.01)

    def test_zero_a_max_raises_error(self):
        """Test that zero a_max raises ValueError."""
        q_start = [0.0] * 6
        q_end = [1.0] * 6
        v_max = [1.0] * 6
        a_max = [0.0] * 6

        with pytest.raises(ValueError, match="All a_max values must be finite and positive"):
            generate_trapezoidal_trajectory(q_start, q_end, v_max, a_max, 0.01)

    def test_negative_dt_raises_error(self):
        """Test that negative dt raises ValueError."""
        q_start = [0.0] * 6
        q_end = [1.0] * 6
        v_max = [1.0] * 6
        a_max = [2.0] * 6

        with pytest.raises(ValueError, match="dt must be finite and positive"):
            generate_trapezoidal_trajectory(q_start, q_end, v_max, a_max, -0.01)

    def test_zero_dt_raises_error(self):
        """Test that zero dt raises ValueError."""
        q_start = [0.0] * 6
        q_end = [1.0] * 6
        v_max = [1.0] * 6
        a_max = [2.0] * 6

        with pytest.raises(ValueError, match="dt must be finite and positive"):
            generate_trapezoidal_trajectory(q_start, q_end, v_max, a_max, 0.0)


class TestTrajectoryPoint:
    """Tests for TrajectoryPoint dataclass."""

    def test_trajectory_point_creation(self):
        """Test that TrajectoryPoint can be created with correct values."""
        point = TrajectoryPoint(
            positions=[0.0, 1.0, 2.0],
            velocities=[0.1, 0.2, 0.3],
            time_from_start=1.5,
        )

        assert point.positions == [0.0, 1.0, 2.0]
        assert point.velocities == [0.1, 0.2, 0.3]
        assert point.time_from_start == 1.5
