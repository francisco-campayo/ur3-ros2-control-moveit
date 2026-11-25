"""Trajectory generation utilities with trapezoidal velocity profiles.

Pure Python module. No ROS 2 dependencies.
"""

from dataclasses import dataclass
from typing import List, Tuple
import math


@dataclass
class TrajectoryPoint:
    """Single point in a trajectory."""
    positions: List[float]
    velocities: List[float]
    time_from_start: float


@dataclass
class _ProfileParameters:
    """Internal parameters of a synchronized velocity profile."""
    distance: float
    direction: float
    acceleration: float
    t_acc: float
    t_const: float
    t_dec: float
    v_peak: float
    total_time: float


def generate_trapezoidal_trajectory(
    q_start: List[float],
    q_end: List[float],
    v_max: List[float],
    a_max: List[float],
    dt: float,
) -> List[TrajectoryPoint]:
    """Generate a synchronized trapezoidal/triangular trajectory.

    Every joint starts and finishes at the same time. Individual joints may
    use a reduced acceleration or velocity so that they synchronize with
    the slowest joint.

    Args:
        q_start: Initial joint positions [rad].
        q_end: Final joint positions [rad].
        v_max: Maximum joint velocity [rad/s].
        a_max: Maximum joint acceleration [rad/s^2].
        dt: Sampling period [s].

    Returns:
        List of TrajectoryPoint.

    Raises:
        ValueError: If inputs are invalid.
    """

    _validate_inputs(q_start, q_end, v_max, a_max, dt)

    num_joints = len(q_start)

    displacements = [
        q_end[i] - q_start[i]
        for i in range(num_joints)
    ]

    # Minimum time required by every joint independently.
    joint_min_times = [
        _calculate_minimum_time(
            abs(displacements[i]),
            v_max[i],
            a_max[i],
        )
        for i in range(num_joints)
    ]

    total_time = max(joint_min_times, default=0.0)

    # Already at target.
    if total_time <= 1e-12:
        return [
            TrajectoryPoint(
                positions=list(q_start),
                velocities=[0.0] * num_joints,
                time_from_start=0.0,
            )
        ]

    # Build one synchronized profile for every joint.
    profiles = [
        _build_profile_for_duration(
            displacement=displacements[i],
            total_time=total_time,
            v_max=v_max[i],
            a_max=a_max[i],
        )
        for i in range(num_joints)
    ]

    # Number of samples.
    num_steps = math.ceil(total_time / dt)

    trajectory: List[TrajectoryPoint] = []

    for step in range(num_steps + 1):
        t = min(step * dt, total_time)

        positions: List[float] = []
        velocities: List[float] = []

        for i in range(num_joints):
            position, velocity = _evaluate_profile(
                q_start=q_start[i],
                profile=profiles[i],
                t=t,
            )

            positions.append(position)
            velocities.append(velocity)

        trajectory.append(
            TrajectoryPoint(
                positions=positions,
                velocities=velocities,
                time_from_start=t,
            )
        )

    # Force the final point to be exactly q_end.
    trajectory[-1] = TrajectoryPoint(
        positions=list(q_end),
        velocities=[0.0] * num_joints,
        time_from_start=total_time,
    )

    return trajectory


def _validate_inputs(
    q_start: List[float],
    q_end: List[float],
    v_max: List[float],
    a_max: List[float],
    dt: float,
) -> None:
    """Validate trajectory inputs."""

    if len(q_start) != len(q_end):
        raise ValueError(
            "q_start and q_end must have the same length"
        )

    if len(v_max) != len(q_start):
        raise ValueError(
            "v_max must have the same length as q_start"
        )

    if len(a_max) != len(q_start):
        raise ValueError(
            "a_max must have the same length as q_start"
        )

    if not q_start:
        raise ValueError("At least one joint is required")

    if any(not math.isfinite(x) for x in q_start):
        raise ValueError("q_start contains non-finite values")

    if any(not math.isfinite(x) for x in q_end):
        raise ValueError("q_end contains non-finite values")

    if any(v <= 0.0 or not math.isfinite(v) for v in v_max):
        raise ValueError(
            "All v_max values must be finite and positive"
        )

    if any(a <= 0.0 or not math.isfinite(a) for a in a_max):
        raise ValueError(
            "All a_max values must be finite and positive"
        )

    if dt <= 0.0 or not math.isfinite(dt):
        raise ValueError("dt must be finite and positive")


def _calculate_minimum_time(
    distance: float,
    v_max: float,
    a_max: float,
) -> float:
    """Calculate the minimum time for one joint."""

    if distance <= 1e-12:
        return 0.0

    t_acc = v_max / a_max
    d_acc = 0.5 * a_max * t_acc ** 2

    # Triangular profile.
    if 2.0 * d_acc >= distance:
        return 2.0 * math.sqrt(distance / a_max)

    # Trapezoidal profile.
    d_const = distance - 2.0 * d_acc
    t_const = d_const / v_max

    return 2.0 * t_acc + t_const


def _build_profile_for_duration(
    displacement: float,
    total_time: float,
    v_max: float,
    a_max: float,
) -> _ProfileParameters:
    """Create a feasible symmetric profile lasting total_time."""

    distance = abs(displacement)

    if distance <= 1e-12:
        return _ProfileParameters(
            distance=0.0,
            direction=1.0,
            acceleration=0.0,
            t_acc=0.0,
            t_const=0.0,
            t_dec=0.0,
            v_peak=0.0,
            total_time=total_time,
        )

    direction = 1.0 if displacement > 0.0 else -1.0

    minimum_time = _calculate_minimum_time(
        distance,
        v_max,
        a_max,
    )

    if total_time < minimum_time - 1e-10:
        raise ValueError(
            "Requested synchronized duration is shorter than "
            "the minimum feasible duration"
        )

    # ---------------------------------------------------------
    # Candidate 1: triangular profile
    #
    # distance = 1/4 * a * T²
    # a = 4d/T²
    # v_peak = 2d/T
    # ---------------------------------------------------------

    acceleration = 4.0 * distance / (total_time ** 2)
    v_peak = 2.0 * distance / total_time

    if (
        acceleration <= a_max + 1e-10
        and v_peak <= v_max + 1e-10
    ):
        t_acc = total_time / 2.0

        return _ProfileParameters(
            distance=distance,
            direction=direction,
            acceleration=acceleration,
            t_acc=t_acc,
            t_const=0.0,
            t_dec=t_acc,
            v_peak=v_peak,
            total_time=total_time,
        )

    # ---------------------------------------------------------
    # Candidate 2: trapezoidal profile using maximum
    # acceleration but reduced peak velocity.
    #
    # d = a * t_acc * (T - t_acc)
    # ---------------------------------------------------------

    discriminant = (
        total_time ** 2
        - 4.0 * distance / a_max
    )

    if discriminant >= -1e-10:
        discriminant = max(0.0, discriminant)

        v_peak = (
            a_max * total_time
            - math.sqrt(discriminant) * a_max
        ) / 2.0

        if v_peak <= v_max + 1e-10:
            t_acc = v_peak / a_max
            t_const = total_time - 2.0 * t_acc

            if t_const >= -1e-10:
                return _ProfileParameters(
                    distance=distance,
                    direction=direction,
                    acceleration=a_max,
                    t_acc=t_acc,
                    t_const=max(0.0, t_const),
                    t_dec=t_acc,
                    v_peak=v_peak,
                    total_time=total_time,
                )

    # ---------------------------------------------------------
    # Candidate 3: maximum velocity with reduced acceleration.
    # ---------------------------------------------------------

    denominator = v_max * total_time - distance

    if denominator <= 0.0:
        raise ValueError(
            "Unable to construct a feasible synchronized profile"
        )

    acceleration = (v_max ** 2) / denominator

    if acceleration > a_max + 1e-8:
        raise ValueError(
            "Unable to construct a feasible synchronized profile"
        )

    v_peak = v_max
    t_acc = v_peak / acceleration
    t_const = total_time - 2.0 * t_acc

    if t_const < -1e-8:
        raise ValueError(
            "Unable to construct a feasible synchronized profile"
        )

    return _ProfileParameters(
        distance=distance,
        direction=direction,
        acceleration=acceleration,
        t_acc=t_acc,
        t_const=max(0.0, t_const),
        t_dec=t_acc,
        v_peak=v_peak,
        total_time=total_time,
    )


def _evaluate_profile(
    q_start: float,
    profile: _ProfileParameters,
    t: float,
) -> Tuple[float, float]:
    """Evaluate one synchronized profile."""

    if profile.distance <= 1e-12:
        return q_start, 0.0

    if t <= 0.0:
        return q_start, 0.0

    if t >= profile.total_time:
        return (
            q_start + profile.direction * profile.distance,
            0.0,
        )

    a = profile.acceleration
    ta = profile.t_acc
    tc = profile.t_const
    v = profile.v_peak
    td_start = ta + tc

    # Acceleration.
    if t <= ta:
        distance = 0.5 * a * t ** 2
        velocity = a * t

    # Constant velocity.
    elif t <= td_start:
        distance_acc = 0.5 * a * ta ** 2

        distance = (
            distance_acc
            + v * (t - ta)
        )

        velocity = v

    # Deceleration.
    else:
        tau = t - td_start

        distance_acc = 0.5 * a * ta ** 2
        distance_const = v * tc

        distance = (
            distance_acc
            + distance_const
            + v * tau
            - 0.5 * a * tau ** 2
        )

        velocity = max(0.0, v - a * tau)

    return (
        q_start + profile.direction * distance,
        profile.direction * velocity,
    )