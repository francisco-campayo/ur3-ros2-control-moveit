"""Singularity and joint-limit checking utilities.

The kinematic model is injected from ur_kinematics.py. This keeps the
mathematical logic independent from ROS 2 package discovery.
"""

from typing import Dict, List, Optional, Tuple

import math

import numpy as np

from my_ur_control.ur_kinematics import URKinematicsModel


# ---------------------------------------------------------------------------
# UR3 joint limits
# ---------------------------------------------------------------------------

UR3_JOINT_ORDER = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


# Mirrors ur_description/config/ur3/joint_limits.yaml (manufacturer data).
# elbow_joint is limited to +-pi there: physically the shoulder_lift link
# prevents the elbow from rotating further, and MoveIt/OMPL uses that range.
UR3_JOINT_LIMITS = {
    "shoulder_pan_joint": (-2.0 * math.pi, 2.0 * math.pi),
    "shoulder_lift_joint": (-2.0 * math.pi, 2.0 * math.pi),
    "elbow_joint": (-math.pi, math.pi),
    "wrist_1_joint": (-2.0 * math.pi, 2.0 * math.pi),
    "wrist_2_joint": (-2.0 * math.pi, 2.0 * math.pi),
    # UR3 wrist_3 has no position limit (infinite rotation, has_position_limits:
    # false). +-2pi is kept as a deliberate, conservative application limit.
    "wrist_3_joint": (-2.0 * math.pi, 2.0 * math.pi),
}


UR3_HOME_POSITION = [
    0.0,
    -1.57,
    1.57,
    -1.57,
    -1.57,
    0.0,
]


# ---------------------------------------------------------------------------
# Jacobian scaling
# ---------------------------------------------------------------------------
#
# The linear part of a geometric Jacobian has units of m/rad.
# The angular part has units of 1/rad.
#
# We normalize the linear rows using a characteristic robot length before
# calculating singular values.
# ---------------------------------------------------------------------------

UR3_CHARACTERISTIC_LENGTH = 0.5


def check_joint_limits(
    q: List[float],
    joint_limits: Optional[dict] = None,
    margin: float = 0.1,
) -> Tuple[bool, str]:
    """Check whether all joint positions are inside safe software limits."""

    if joint_limits is None:
        joint_limits = UR3_JOINT_LIMITS

    if len(q) != 6:
        return False, f"Expected 6 joints, got {len(q)}"

    if margin < 0.0:
        return False, "Margin cannot be negative"

    for i, joint_name in enumerate(UR3_JOINT_ORDER):

        if joint_name not in joint_limits:
            return False, (
                f"No limits configured for '{joint_name}'"
            )

        min_limit, max_limit = joint_limits[joint_name]

        q_min = min_limit + margin
        q_max = max_limit - margin

        if q[i] < q_min or q[i] > q_max:
            return False, (
                f"Joint '{joint_name}' at {q[i]:.3f} rad is outside "
                f"safe limits [{q_min:.3f}, {q_max:.3f}] rad"
            )

    return True, ""


def compute_jacobian(
    q: List[float],
    model: URKinematicsModel,
) -> List[List[float]]:
    """Compute the calibrated UR3 geometric Jacobian.

    The actual numerical kinematics come from ur_description.
    """

    J = model.compute_jacobian(q)

    return J.tolist()


def compute_singular_values(
    q: List[float],
    model: URKinematicsModel,
    characteristic_length: float = UR3_CHARACTERISTIC_LENGTH,
) -> List[float]:
    """Compute singular values of a dimensionally scaled Jacobian."""

    if characteristic_length <= 0.0:
        raise ValueError(
            "characteristic_length must be positive"
        )

    J = model.compute_jacobian(q)

    # ------------------------------------------------------------------
    # Normalize linear rows.
    # ------------------------------------------------------------------

    J_scaled = J.copy()

    J_scaled[:3, :] /= characteristic_length

    # ------------------------------------------------------------------
    # NumPy SVD.
    #
    # This is much more numerically robust than calculating det(J).
    # ------------------------------------------------------------------

    singular_values = np.linalg.svd(
        J_scaled,
        compute_uv=False,
    )

    return singular_values.tolist()


def check_singularity(
    q: List[float],
    model: URKinematicsModel,
    sigma_min_threshold: float = 0.02,
    characteristic_length: float = UR3_CHARACTERISTIC_LENGTH,
) -> Tuple[bool, str]:
    """Check whether the robot is near a kinematic singularity."""

    if len(q) != 6:
        return False, (
            f"Expected 6 joints, got {len(q)}"
        )

    if sigma_min_threshold <= 0.0:
        return False, (
            "sigma_min_threshold must be positive"
        )

    singular_values = compute_singular_values(
        q=q,
        model=model,
        characteristic_length=characteristic_length,
    )

    sigma_max = singular_values[0]
    sigma_min = singular_values[-1]

    if sigma_min <= 1e-15:
        condition_number = math.inf
    else:
        condition_number = (
            sigma_max / sigma_min
        )

    if sigma_min < sigma_min_threshold:
        return True, (
            "Near singular configuration: "
            f"sigma_min={sigma_min:.6e}, "
            f"threshold={sigma_min_threshold:.6e}, "
            f"condition_number={condition_number:.3e}"
        )

    return False, (
        "Configuration OK: "
        f"sigma_min={sigma_min:.6e}, "
        f"condition_number={condition_number:.3e}"
    )


def check_trajectory_for_singularities(
    trajectory_positions: List[List[float]],
    model: URKinematicsModel,
    trajectory_times: Optional[List[float]] = None,
    sigma_min_threshold: float = 0.02,
    characteristic_length: float = UR3_CHARACTERISTIC_LENGTH,
) -> Tuple[bool, str]:
    """Check every sampled point of a trajectory for singularities."""

    if not trajectory_positions:
        return False, "Trajectory is empty"

    if trajectory_times is not None:
        if len(trajectory_times) != len(trajectory_positions):
            return False, (
                "trajectory_times and trajectory_positions "
                "must have the same length"
            )

    minimum_sigma = math.inf
    minimum_index = -1

    for index, q in enumerate(
        trajectory_positions
    ):

        singular, message = check_singularity(
            q=q,
            model=model,
            sigma_min_threshold=sigma_min_threshold,
            characteristic_length=characteristic_length,
        )

        sigma_values = compute_singular_values(
            q=q,
            model=model,
            characteristic_length=characteristic_length,
        )

        sigma_min = sigma_values[-1]

        if sigma_min < minimum_sigma:
            minimum_sigma = sigma_min
            minimum_index = index

        if singular:

            if trajectory_times is not None:
                return True, (
                    "Trajectory rejected: near singularity at "
                    f"point {index}, "
                    f"t={trajectory_times[index]:.3f}s. "
                    f"{message}"
                )

            return True, (
                "Trajectory rejected: near singularity at "
                f"point {index}. "
                f"{message}"
            )

    if trajectory_times is not None:
        return False, (
            "Trajectory singularity check passed. "
            f"Minimum sigma_min={minimum_sigma:.6e} "
            f"at point {minimum_index}, "
            f"t={trajectory_times[minimum_index]:.3f}s."
        )

    return False, (
        "Trajectory singularity check passed. "
        f"Minimum sigma_min={minimum_sigma:.6e} "
        f"at point {minimum_index}."
    )


def is_at_home(
    q: List[float],
    home_position: Optional[List[float]] = None,
    tolerance: float = 0.01,
) -> bool:
    """Check whether the robot is at Home."""

    if home_position is None:
        home_position = UR3_HOME_POSITION

    if len(q) != len(home_position):
        return False

    return all(
        abs(q[i] - home_position[i]) <= tolerance
        for i in range(len(q))
    )


def get_joint_limits_with_margin(
    margin: float = 0.1,
) -> Dict[str, Tuple[float, float]]:
    """Return software limits with a safety margin."""

    if margin < 0.0:
        raise ValueError(
            "margin cannot be negative"
        )

    result = {}

    for joint_name, (
        min_limit,
        max_limit,
    ) in UR3_JOINT_LIMITS.items():

        result[joint_name] = (
            min_limit + margin,
            max_limit - margin,
        )

    return result