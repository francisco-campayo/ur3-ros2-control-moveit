"""Offline test for UR3 trajectory singularities.

This script does NOT require ROS 2, Gazebo or any running node.

It:
1. Loads the UR3 kinematic model from default_kinematics.yaml.
2. Generates the same trapezoidal trajectory used by the Go Home service.
3. Evaluates the Jacobian at every trajectory point.
4. Calculates sigma_min and the condition number.
5. Reports the worst point of the trajectory.
6. Allows the singularity threshold to be selected from the command line.

Default threshold:
    0.02

Examples:

    python3 test_trajectory_singularity.py

    python3 test_trajectory_singularity.py --threshold 0.01

    python3 test_trajectory_singularity.py --threshold 0.005

    python3 test_trajectory_singularity.py --q \
        0.613139 -0.367991 -0.717982 -1.226637 -0.735982 0.188496
"""

import argparse
import math
from pathlib import Path

import numpy as np
import pytest

from my_ur_control.trajectory_utils import (
    generate_trapezoidal_trajectory,
)

from my_ur_control.ur_kinematics import (
    URKinematicsModel,
    find_kinematics_file,
)


# ============================================================================
# Configuration
# ============================================================================

DEFAULT_THRESHOLD = 0.02

CHARACTERISTIC_LENGTH = 0.5

DT = 0.01

V_MAX = [
    3.1416,
    3.1416,
    3.1416,
    6.2832,
    6.2832,
    6.2832,
]

A_MAX = [
    4.0,
    4.0,
    4.0,
    8.0,
    8.0,
    8.0,
]


HOME_POSITION = [
    0.0,
    -1.57,
    1.57,
    -1.57,
    -1.57,
    0.0,
]


# ============================================================================
# Configurations from the screenshots
# ============================================================================

CAPTURE_1 = [
    0.245327,
    -1.471964,
    -0.418823,
    -1.226637,
    0.0,
    0.0,
]


CAPTURE_2 = [
    0.613139,
    -0.367991,
    -0.717982,
    -1.226637,
    -0.735982,
    0.188496,
]


# ============================================================================
# Kinematics file
# ============================================================================

# Resolved via UR_KINEMATICS_FILE, the installed ur_description package
# (ament_index) or the legacy ~/ur_ws/src fallback.
KINEMATICS_FILE = find_kinematics_file("ur3")


# ============================================================================
# Jacobian / singularity calculation
# ============================================================================

def compute_sigma_values(
    model: URKinematicsModel,
    q: list[float],
) -> tuple[np.ndarray, float, float]:
    """Calculate scaled singular values, sigma_min and condition number."""

    J = model.compute_jacobian(q)

    J_scaled = J.copy()

    # The linear part has units of metres/rad.
    # Normalize it using the same characteristic length used
    # by singularity_utils.py.
    J_scaled[:3, :] /= CHARACTERISTIC_LENGTH

    singular_values = np.linalg.svd(
        J_scaled,
        compute_uv=False,
    )

    sigma_max = singular_values[0]
    sigma_min = singular_values[-1]

    if sigma_min <= 1e-15:
        condition_number = math.inf
    else:
        condition_number = (
            sigma_max / sigma_min
        )

    return (
        singular_values,
        sigma_min,
        condition_number,
    )


# ============================================================================
# Print helpers
# ============================================================================

def print_configuration(
    q: list[float],
) -> None:
    """Print a joint configuration."""

    print(
        "["
        + ", ".join(
            f"{value: .6f}"
            for value in q
        )
        + "]"
    )


def print_result_line(
    index: int,
    time_from_start: float,
    sigma_min: float,
    condition_number: float,
    threshold: float,
) -> None:
    """Print one trajectory result."""

    if condition_number == math.inf:
        condition_text = "inf"
    else:
        condition_text = f"{condition_number:.3e}"

    status = (
        "SINGULAR / REJECT"
        if sigma_min < threshold
        else "OK"
    )

    print(
        f"  {index:4d} | "
        f"t={time_from_start:7.3f}s | "
        f"sigma_min={sigma_min:.6e} | "
        f"condition={condition_text:>10s} | "
        f"{status}"
    )


# ============================================================================
# Fixtures
# ============================================================================

@pytest.fixture
def model() -> URKinematicsModel:
    """Load the UR3 kinematic model from YAML."""
    return URKinematicsModel.from_yaml(KINEMATICS_FILE)


# ============================================================================
# Test one trajectory
# ============================================================================

def run_trajectory_analysis(
    model: URKinematicsModel,
    name: str,
    q_start: list[float],
    threshold: float,
) -> None:
    """Generate and analyse one trajectory."""

    print()
    print("=" * 90)
    print(f"TEST: {name}")
    print("=" * 90)

    print()
    print("Initial configuration:")
    print_configuration(q_start)

    print()
    print("Home configuration:")
    print_configuration(HOME_POSITION)

    # ------------------------------------------------------------------
    # Initial configuration analysis
    # ------------------------------------------------------------------

    (
        initial_singular_values,
        initial_sigma_min,
        initial_condition,
    ) = compute_sigma_values(
        model,
        q_start,
    )

    print()
    print("Initial configuration analysis:")
    print(
        f"  sigma_min       = "
        f"{initial_sigma_min:.6e}"
    )

    if initial_condition == math.inf:
        print(
            "  condition number = inf"
        )
    else:
        print(
            f"  condition number = "
            f"{initial_condition:.6e}"
        )

    if initial_sigma_min < threshold:
        print(
            "  RESULT            = "
            "INITIAL CONFIGURATION REJECTED"
        )
    else:
        print(
            "  RESULT            = "
            "INITIAL CONFIGURATION OK"
        )

    # ------------------------------------------------------------------
    # Generate trajectory
    # ------------------------------------------------------------------

    print()
    print("Generating trapezoidal trajectory...")

    try:
        trajectory = (
            generate_trapezoidal_trajectory(
                q_start=q_start,
                q_end=HOME_POSITION,
                v_max=V_MAX,
                a_max=A_MAX,
                dt=DT,
            )
        )

    except Exception as exc:
        print()
        print(
            "ERROR generating trajectory:"
        )
        print(
            f"  {exc}"
        )
        return

    if not trajectory:
        print()
        print(
            "ERROR: trajectory is empty."
        )
        return

    print(
        f"  Number of points = "
        f"{len(trajectory)}"
    )

    print(
        f"  Duration         = "
        f"{trajectory[-1].time_from_start:.3f} s"
    )

    # ------------------------------------------------------------------
    # Analyse every trajectory point
    # ------------------------------------------------------------------

    results = []

    for index, point in enumerate(
        trajectory
    ):

        (
            singular_values,
            sigma_min,
            condition_number,
        ) = compute_sigma_values(
            model,
            point.positions,
        )

        results.append(
            {
                "index": index,
                "time": point.time_from_start,
                "positions": point.positions,
                "sigma_min": sigma_min,
                "condition_number": condition_number,
                "singular_values": singular_values,
            }
        )

    # ------------------------------------------------------------------
    # Find minimum sigma
    # ------------------------------------------------------------------

    worst_point = min(
        results,
        key=lambda result: result["sigma_min"],
    )

    minimum_sigma = (
        worst_point["sigma_min"]
    )

    minimum_index = (
        worst_point["index"]
    )

    minimum_time = (
        worst_point["time"]
    )

    minimum_condition = (
        worst_point["condition_number"]
    )

    # ------------------------------------------------------------------
    # Print trajectory summary
    # ------------------------------------------------------------------

    print()
    print("-" * 90)
    print("TRAJECTORY SUMMARY")
    print("-" * 90)

    print(
        f"  Threshold             = "
        f"{threshold:.6e}"
    )

    print(
        f"  Minimum sigma_min     = "
        f"{minimum_sigma:.6e}"
    )

    print(
        f"  Minimum at point      = "
        f"{minimum_index}"
    )

    print(
        f"  Minimum at time       = "
        f"{minimum_time:.3f} s"
    )

    if minimum_condition == math.inf:
        print(
            "  Condition number      = inf"
        )
    else:
        print(
            f"  Condition number      = "
            f"{minimum_condition:.6e}"
        )

    # ------------------------------------------------------------------
    # Overall result
    # ------------------------------------------------------------------

    if minimum_sigma < threshold:

        print()
        print(
            "  >>> TRAJECTORY: REJECTED <<<"
        )

        print(
            "  A trajectory point is below "
            "the selected threshold."
        )

    else:

        print()
        print(
            "  >>> TRAJECTORY: ACCEPTED <<<"
        )

        print(
            "  No trajectory point is below "
            "the selected threshold."
        )

    # ------------------------------------------------------------------
    # Print worst point
    # ------------------------------------------------------------------

    print()
    print("-" * 90)
    print("WORST TRAJECTORY POINT")
    print("-" * 90)

    print(
        f"  Point: {minimum_index}"
    )

    print(
        f"  Time : {minimum_time:.3f} s"
    )

    print(
        "  q:"
    )

    print_configuration(
        worst_point["positions"]
    )

    print()
    print(
        "  Singular values:"
    )

    print(
        "  "
        + " ".join(
            f"{value:.6e}"
            for value in worst_point[
                "singular_values"
            ]
        )
    )

    # ------------------------------------------------------------------
    # Show points around the minimum
    # ------------------------------------------------------------------

    print()
    print("-" * 90)
    print("POINTS AROUND THE MINIMUM")
    print("-" * 90)

    first_index = max(
        0,
        minimum_index - 5,
    )

    last_index = min(
        len(results),
        minimum_index + 6,
    )

    print(
        "  Point | Time       | sigma_min        | "
        "Condition  | Status"
    )

    print(
        "  "
        + "-" * 82
    )

    for result in results[
        first_index:last_index
    ]:

        print_result_line(
            index=result["index"],
            time_from_start=result["time"],
            sigma_min=result["sigma_min"],
            condition_number=result[
                "condition_number"
            ],
            threshold=threshold,
        )

    # ------------------------------------------------------------------
    # Threshold sensitivity
    #
    # This is useful for understanding whether 0.02 is simply too
    # restrictive. It does NOT change the selected threshold.
    # ------------------------------------------------------------------

    comparison_thresholds = [
        0.050,
        0.020,
        0.015,
        0.010,
        0.005,
        0.001,
    ]

    print()
    print("-" * 90)
    print("THRESHOLD SENSITIVITY")
    print("-" * 90)

    print(
        "  This comparison is only informative."
    )

    print()

    for comparison_threshold in (
        comparison_thresholds
    ):

        rejected = (
            minimum_sigma
            < comparison_threshold
        )

        result_text = (
            "REJECT"
            if rejected
            else "ACCEPT"
        )

        print(
            f"  threshold={comparison_threshold:.3f}"
            f"  -> {result_text}"
        )


# ============================================================================
# Argument parser
# ============================================================================

def parse_arguments():
    """Parse command-line arguments."""

    parser = argparse.ArgumentParser(
        description=(
            "Offline UR3 trajectory singularity "
            "test."
        )
    )

    parser.add_argument(
        "--threshold",
        type=float,
        default=DEFAULT_THRESHOLD,
        help=(
            "Minimum sigma_min considered safe. "
            f"Default: {DEFAULT_THRESHOLD}"
        ),
    )

    parser.add_argument(
        "--q",
        nargs=6,
        type=float,
        metavar=(
            "Q1",
            "Q2",
            "Q3",
            "Q4",
            "Q5",
            "Q6",
        ),
        help=(
            "Custom initial joint configuration. "
            "If omitted, the two screenshot "
            "configurations are tested."
        ),
    )

    return parser.parse_args()


# ============================================================================
# Main
# ============================================================================

def main() -> None:

    args = parse_arguments()

    # ------------------------------------------------------------------
    # Validate threshold
    # ------------------------------------------------------------------

    if args.threshold <= 0.0:

        raise ValueError(
            "Threshold must be greater than zero."
        )

    threshold = args.threshold

    # ------------------------------------------------------------------
    # Header
    # ------------------------------------------------------------------

    print()
    print("=" * 90)
    print("UR3 OFFLINE TRAJECTORY SINGULARITY TEST")
    print("=" * 90)

    print()
    print(
        f"Singularity threshold: "
        f"{threshold:.6e}"
    )

    print(
        f"Characteristic length: "
        f"{CHARACTERISTIC_LENGTH:.3f} m"
    )

    print(
        f"Trajectory dt: "
        f"{DT:.3f} s"
    )

    print()
    print(
        f"Kinematics file:"
    )

    print(
        f"  {KINEMATICS_FILE}"
    )

    # ------------------------------------------------------------------
    # Check YAML
    # ------------------------------------------------------------------

    if not KINEMATICS_FILE.is_file():

        raise FileNotFoundError(
            "Could not find kinematics file:\n"
            f"{KINEMATICS_FILE}"
        )

    # ------------------------------------------------------------------
    # Load model
    # ------------------------------------------------------------------

    print()
    print(
        "Loading UR3 kinematic model..."
    )

    model = URKinematicsModel.from_yaml(
        KINEMATICS_FILE
    )

    print(
        "Model loaded successfully."
    )

    print(
        f"Calibration hash: "
        f"{model.calibration_hash}"
    )

    # ------------------------------------------------------------------
    # Custom configuration
    # ------------------------------------------------------------------

    if args.q is not None:

        q_custom = list(
            args.q
        )

        run_trajectory_analysis(
            model=model,
            name="CUSTOM CONFIGURATION",
            q_start=q_custom,
            threshold=threshold,
        )

    # ------------------------------------------------------------------
    # Default configurations
    # ------------------------------------------------------------------

    else:

        run_trajectory_analysis(
            model=model,
            name="CAPTURE 1",
            q_start=CAPTURE_1,
            threshold=threshold,
        )

        run_trajectory_analysis(
            model=model,
            name="CAPTURE 2",
            q_start=CAPTURE_2,
            threshold=threshold,
        )

    # ------------------------------------------------------------------
    # End
    # ------------------------------------------------------------------

    print()
    print("=" * 90)
    print("TEST FINISHED")
    print("=" * 90)
    print()


# ============================================================================
# Pytest tests
# ============================================================================

def test_capture_1(model: URKinematicsModel) -> None:
    """Test trajectory singularity analysis for Capture 1."""
    run_trajectory_analysis(
        model=model,
        name="CAPTURE 1",
        q_start=CAPTURE_1,
        threshold=DEFAULT_THRESHOLD,
    )


def test_capture_2(model: URKinematicsModel) -> None:
    """Test trajectory singularity analysis for Capture 2."""
    run_trajectory_analysis(
        model=model,
        name="CAPTURE 2",
        q_start=CAPTURE_2,
        threshold=DEFAULT_THRESHOLD,
    )


if __name__ == "__main__":
    main()
