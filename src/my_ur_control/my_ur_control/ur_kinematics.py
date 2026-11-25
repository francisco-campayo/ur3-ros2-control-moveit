"""UR robot kinematics based directly on ur_description calibration YAML.

This module intentionally contains no ROS 2 dependencies.

The kinematic chain follows the same structure used by the Universal Robots
ROS 2 description:

    base_link
        |
        +-- base_link_inertia
        |
        +-- shoulder_pan_joint
        +-- shoulder_lift_joint
        +-- elbow_joint
        +-- wrist_1_joint
        +-- wrist_2_joint
        +-- wrist_3_joint
        |
        +-- flange
        |
        +-- tool0

The joint origins and orientations are loaded directly from
default_kinematics.yaml instead of being converted into a nominal DH table.
"""

from dataclasses import dataclass
import os
from pathlib import Path
from typing import List, Sequence, Tuple

import math

import numpy as np
import yaml


UR3_JOINT_KEYS = (
    "shoulder",
    "upper_arm",
    "forearm",
    "wrist_1",
    "wrist_2",
    "wrist_3",
)

_NUM_JOINTS = 6


# Environment variable that overrides the kinematics file location.
KINEMATICS_FILE_ENV = "UR_KINEMATICS_FILE"


def find_kinematics_file(ur_type: str = "ur3") -> Path:
    """Locate ``default_kinematics.yaml`` for the given UR model.

    Resolution order:

    1. Environment variable ``UR_KINEMATICS_FILE`` (explicit override).
    2. Installed ``ur_description`` package via ament_index (requires a
       sourced ROS 2 workspace). Imported lazily so this module keeps
       working without ROS 2.
    3. Legacy fallback: ``~/ur_ws/src/Universal_Robots_ROS2_Description``.

    The first candidate that exists is returned. If none exists, the
    best-effort candidate is returned so the caller gets a clear
    FileNotFoundError from ``URKinematicsModel.from_yaml``.
    """

    relative = Path("config") / ur_type / "default_kinematics.yaml"
    candidates: List[Path] = []

    env_path = os.environ.get(KINEMATICS_FILE_ENV)
    if env_path:
        candidates.append(Path(env_path).expanduser())

    try:
        from ament_index_python.packages import (
            PackageNotFoundError,
            get_package_share_directory,
        )

        try:
            candidates.append(
                Path(get_package_share_directory("ur_description")) / relative
            )
        except PackageNotFoundError:
            pass
    except ImportError:
        pass

    candidates.append(
        Path.home() / "ur_ws" / "src"
        / "Universal_Robots_ROS2_Description" / relative
    )

    for candidate in candidates:
        if candidate.is_file():
            return candidate

    return candidates[0]


@dataclass(frozen=True)
class Pose:
    """Rigid transform represented by xyz + roll/pitch/yaw."""

    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float

    def as_matrix(self) -> np.ndarray:
        """Convert this pose to a 4x4 homogeneous transformation."""

        T = np.eye(4, dtype=float)

        T[:3, :3] = rpy_to_rotation_matrix(
            self.roll,
            self.pitch,
            self.yaw,
        )

        T[:3, 3] = [
            self.x,
            self.y,
            self.z,
        ]

        return T


@dataclass
class URKinematicsModel:
    """Kinematic model loaded from a Universal Robots YAML calibration."""

    joint_origins: Tuple[Pose, ...]
    calibration_hash: str
    source_file: str

    @classmethod
    def from_yaml(
        cls,
        yaml_path: str | Path,
    ) -> "URKinematicsModel":
        """Load a UR kinematic model from default_kinematics.yaml."""

        path = Path(yaml_path).expanduser().resolve()

        if not path.is_file():
            raise FileNotFoundError(
                f"Kinematics file does not exist: {path}"
            )

        with path.open("r", encoding="utf-8") as file:
            data = yaml.safe_load(file)

        if not isinstance(data, dict):
            raise ValueError(
                "Kinematics YAML root must be a dictionary"
            )

        kinematics = data.get("kinematics")

        if not isinstance(kinematics, dict):
            raise ValueError(
                "YAML does not contain a 'kinematics' section"
            )

        joint_origins = []

        for key in UR3_JOINT_KEYS:

            if key not in kinematics:
                raise ValueError(
                    f"Missing kinematics entry: '{key}'"
                )

            values = kinematics[key]

            required_fields = (
                "x",
                "y",
                "z",
                "roll",
                "pitch",
                "yaw",
            )

            missing = [
                field
                for field in required_fields
                if field not in values
            ]

            if missing:
                raise ValueError(
                    f"Kinematics entry '{key}' is missing "
                    f"fields: {missing}"
                )

            pose = Pose(
                x=float(values["x"]),
                y=float(values["y"]),
                z=float(values["z"]),
                roll=float(values["roll"]),
                pitch=float(values["pitch"]),
                yaw=float(values["yaw"]),
            )

            joint_origins.append(pose)

        calibration_hash = str(
            kinematics.get(
                "hash",
                "unknown",
            )
        )

        return cls(
            joint_origins=tuple(joint_origins),
            calibration_hash=calibration_hash,
            source_file=str(path),
        )

    def compute_jacobian(
        self,
        q: Sequence[float],
    ) -> np.ndarray:
        """Compute the 6x6 geometric Jacobian for tool0.

        Args:
            q:
                Six joint positions in radians.

        Returns:
            6x6 geometric Jacobian:

                [v]
                [ω] = J(q) q_dot

            The first three rows correspond to linear velocity.
            The last three rows correspond to angular velocity.
        """

        q_array = np.asarray(
            q,
            dtype=float,
        )

        if q_array.shape != (6,):
            raise ValueError(
                "Expected exactly six joint positions"
            )

        # ------------------------------------------------------------------
        # base_link -> base_link_inertia
        #
        # The UR ROS description inserts a fixed rotation of pi around Z.
        # ------------------------------------------------------------------

        T = np.eye(4, dtype=float)

        T[:3, :3] = rotation_z(
            math.pi
        )

        joint_axes: List[np.ndarray] = []
        joint_origins: List[np.ndarray] = []

        # ------------------------------------------------------------------
        # Serial chain.
        #
        # In the URDF each joint has:
        #
        #     parent
        #        |
        #        +-- fixed joint origin transform
        #                |
        #                +-- revolute motion around local Z
        #
        # Therefore:
        #
        #     T = T_parent @ T_origin @ Rz(q)
        # ------------------------------------------------------------------

        for i in range(_NUM_JOINTS):

            origin = self.joint_origins[i]

            T_origin = origin.as_matrix()

            # Transform to the joint frame before applying q_i.
            T_joint = T @ T_origin

            # Joint axis is z expressed in the world/base frame.
            z_axis_world = (
                T_joint[:3, :3]
                @ np.array([0.0, 0.0, 1.0])
            )

            joint_axes.append(
                z_axis_world
            )

            # Position of the joint axis.
            joint_origins.append(
                T_joint[:3, 3].copy()
            )

            # Apply the actual joint motion.
            T = T_joint @ rotation_z_homogeneous(
                q_array[i]
            )

        # ------------------------------------------------------------------
        # wrist_3_link -> flange
        # ------------------------------------------------------------------

        flange_transform = Pose(
            x=0.0,
            y=0.0,
            z=0.0,
            roll=0.0,
            pitch=-math.pi / 2.0,
            yaw=-math.pi / 2.0,
        ).as_matrix()

        T = T @ flange_transform

        # ------------------------------------------------------------------
        # flange -> tool0
        # ------------------------------------------------------------------

        tool0_transform = Pose(
            x=0.0,
            y=0.0,
            z=0.0,
            roll=math.pi / 2.0,
            pitch=0.0,
            yaw=math.pi / 2.0,
        ).as_matrix()

        T = T @ tool0_transform

        # Final tool0 position.
        o_tool = T[:3, 3]

        J = np.zeros(
            (6, 6),
            dtype=float,
        )

        for i in range(_NUM_JOINTS):

            z_i = joint_axes[i]
            o_i = joint_origins[i]

            # Linear velocity component:
            #
            #     Jv = z_i x (o_tool - o_i)
            #
            J[:3, i] = np.cross(
                z_i,
                o_tool - o_i,
            )

            # Angular velocity component:
            #
            #     Jw = z_i
            #
            J[3:, i] = z_i

        return J

    def forward_kinematics(
        self,
        q: Sequence[float],
    ) -> np.ndarray:
        """Compute the homogeneous transform from base_link to tool0."""

        q_array = np.asarray(
            q,
            dtype=float,
        )

        if q_array.shape != (6,):
            raise ValueError(
                "Expected exactly six joint positions"
            )

        T = np.eye(4, dtype=float)

        # base_link -> base_link_inertia
        T[:3, :3] = rotation_z(
            math.pi
        )

        for i in range(_NUM_JOINTS):

            T = (
                T
                @ self.joint_origins[i].as_matrix()
                @ rotation_z_homogeneous(q_array[i])
            )

        # wrist_3_link -> flange
        T = T @ Pose(
            x=0.0,
            y=0.0,
            z=0.0,
            roll=0.0,
            pitch=-math.pi / 2.0,
            yaw=-math.pi / 2.0,
        ).as_matrix()

        # flange -> tool0
        T = T @ Pose(
            x=0.0,
            y=0.0,
            z=0.0,
            roll=math.pi / 2.0,
            pitch=0.0,
            yaw=math.pi / 2.0,
        ).as_matrix()

        return T


def rpy_to_rotation_matrix(
    roll: float,
    pitch: float,
    yaw: float,
) -> np.ndarray:
    """Convert URDF roll/pitch/yaw to a rotation matrix.

    Equivalent to:

        R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    """

    cr = math.cos(roll)
    sr = math.sin(roll)

    cp = math.cos(pitch)
    sp = math.sin(pitch)

    cy = math.cos(yaw)
    sy = math.sin(yaw)

    return np.array(
        [
            [
                cy * cp,
                cy * sp * sr - sy * cr,
                cy * sp * cr + sy * sr,
            ],
            [
                sy * cp,
                sy * sp * sr + cy * cr,
                sy * sp * cr - cy * sr,
            ],
            [
                -sp,
                cp * sr,
                cp * cr,
            ],
        ],
        dtype=float,
    )


def rotation_z(
    angle: float,
) -> np.ndarray:
    """Return a 3x3 rotation about Z."""

    c = math.cos(angle)
    s = math.sin(angle)

    return np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def rotation_z_homogeneous(
    angle: float,
) -> np.ndarray:
    """Return a 4x4 rotation about Z."""

    T = np.eye(4, dtype=float)

    T[:3, :3] = rotation_z(angle)

    return T