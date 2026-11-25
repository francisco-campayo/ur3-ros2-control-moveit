"""Go Home service node for UR3.

The node:
1. Reads the current joint state.
2. Validates joint limits.
3. Generates a synchronized trapezoidal joint-space trajectory.
4. Validates the trajectory final position.
5. Publishes the trajectory only after all configured checks pass.
6. Waits until the robot reaches the Home position.

Go Home intentionally does NOT use singularity detection. It is a pure
joint-space motion to a predefined Home configuration. Singularity analysis
remains available as a reusable validator for future Cartesian/Pick-and-Place
trajectories.

The mathematical trajectory modules remain independent of ROS 2.
"""

import threading
import time
from typing import Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from my_ur_control.singularity_utils import (
    UR3_HOME_POSITION,
    check_joint_limits,
    is_at_home,
)

from my_ur_control.trajectory_utils import (
    generate_trapezoidal_trajectory,
)


class GoHomeService(Node):
    """ROS 2 service that moves the UR3 to a predefined Home position."""

    def __init__(self):
        super().__init__("go_home_service")

        # ==================================================================
        # Configuration
        # ==================================================================

        self.home_position = list(UR3_HOME_POSITION)

        self.tolerance = 0.01
        self.timeout = 30.0

        # ------------------------------------------------------------------
        # Trajectory parameters
        #
        # These are application parameters, not necessarily official
        # acceleration limits from Universal Robots.
        # ------------------------------------------------------------------

        self.v_max = [
            3.1416,
            3.1416,
            3.1416,
            6.2832,
            6.2832,
            6.2832,
        ]

        self.a_max = [
            4.0,
            4.0,
            4.0,
            8.0,
            8.0,
            8.0,
        ]

        self.dt = 0.01

        # ==================================================================
        # Joint configuration
        # ==================================================================

        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        # ==================================================================
        # Runtime state
        # ==================================================================

        self.current_positions: Optional[list[float]] = None

        self.is_moving = False
        self.movement_start_time: Optional[float] = None

        # Prevent two service calls from commanding the robot simultaneously.
        self.service_lock = threading.Lock()

        # Reentrant callback group is required because the service callback
        # waits while /joint_states must continue being processed.
        self.callback_group = ReentrantCallbackGroup()

        # ==================================================================
        # ROS interfaces
        # ==================================================================

        self.srv = self.create_service(
            SetBool,
            "go_home",
            self.go_home_callback,
            callback_group=self.callback_group,
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            "/scaled_joint_trajectory_controller/joint_trajectory",
            10,
        )

        # ==================================================================
        # Startup information
        # ==================================================================

        self.get_logger().info(
            "=================================================="
        )

        self.get_logger().info(
            "Go Home service initialized."
        )

        self.get_logger().info(
            f"Home position: {self.home_position}"
        )

        self.get_logger().info(
            f"Trajectory dt: {self.dt:.3f} s"
        )

        self.get_logger().info(
            "Singularity checking: DISABLED for Go Home "
            "(joint-space trajectory)"
        )

        self.get_logger().info(
            "Validation enabled: joint limits + final Home position"
        )

        self.get_logger().info(
            "Waiting for joint states..."
        )

        self.get_logger().info(
            "=================================================="
        )


    # ======================================================================
    # Joint state callback
    # ======================================================================

    def joint_state_callback(
        self,
        msg: JointState,
    ):
        """Update current joint positions from /joint_states."""

        try:
            positions = []

            for joint_name in self.joint_names:
                try:
                    index = msg.name.index(
                        joint_name
                    )

                except ValueError:
                    self.get_logger().debug(
                        f"Joint '{joint_name}' "
                        f"not present in JointState message."
                    )
                    return

                positions.append(
                    float(msg.position[index])
                )

            self.current_positions = positions

        except Exception as exc:
            self.get_logger().warn(
                f"Error processing JointState: {exc}"
            )

    # ======================================================================
    # Service callback
    # ======================================================================

    def go_home_callback(
        self,
        request,
        response,
    ):
        """Handle a Go Home service request."""

        # Only one movement may be commanded at a time.
        with self.service_lock:

            self.get_logger().info(
                "Received go-home request."
            )

            if not request.data:
                response.success = False
                response.message = (
                    "Invalid request: data must be true."
                )
                return response

            if self.is_moving:
                response.success = False
                response.message = (
                    "Robot is already executing a trajectory."
                )
                return response

            success, message = self.execute_go_home()

            response.success = success
            response.message = message

            return response

    # ======================================================================
    # Go Home execution
    # ======================================================================

    def execute_go_home(self):
        """Validate, generate and execute the Go Home trajectory."""

        # ------------------------------------------------------------------
        # 1. Current state
        # ------------------------------------------------------------------

        if self.current_positions is None:
            return (
                False,
                "No joint states available.",
            )

        q_start = list(
            self.current_positions
        )

        self.get_logger().info(
            f"Current joint positions: {q_start}"
        )

        # ------------------------------------------------------------------
        # 2. Already Home
        # ------------------------------------------------------------------

        if is_at_home(
            q_start,
            self.home_position,
            self.tolerance,
        ):
            self.get_logger().info(
                "Robot is already at Home."
            )

            return (
                True,
                "Robot is already at Home position.",
            )

        # ------------------------------------------------------------------
        # 3. Current joint limits
        # ------------------------------------------------------------------

        limits_valid, limits_message = (
            check_joint_limits(
                q_start
            )
        )

        if not limits_valid:
            self.get_logger().error(
                f"Joint limits check failed: "
                f"{limits_message}"
            )

            return (
                False,
                f"Joint limits check failed: "
                f"{limits_message}",
            )

        self.get_logger().info(
            "Current joint limits check passed."
        )

        # ------------------------------------------------------------------
        # 4. Generate trajectory
        # ------------------------------------------------------------------

        self.get_logger().info(
            "Generating trapezoidal trajectory..."
        )

        try:
            trajectory = (
                generate_trapezoidal_trajectory(
                    q_start=q_start,
                    q_end=self.home_position,
                    v_max=self.v_max,
                    a_max=self.a_max,
                    dt=self.dt,
                )
            )

        except Exception as exc:
            self.get_logger().error(
                f"Trajectory generation failed: {exc}"
            )

            return (
                False,
                f"Trajectory generation failed: {exc}",
            )

        if not trajectory:
            return (
                False,
                "Generated trajectory is empty.",
            )

        # ------------------------------------------------------------------
        # 5. Extract trajectory timing
        # ------------------------------------------------------------------

        total_time = trajectory[-1].time_from_start

        self.get_logger().info(
            f"Generated {len(trajectory)} trajectory points."
        )

        self.get_logger().info(
            f"Planned movement duration: "
            f"{total_time:.3f} s"
        )


        # ------------------------------------------------------------------
        # 6. Validate final position
        # ------------------------------------------------------------------

        final_position = (
            trajectory[-1].positions
        )

        final_error = max(
            abs(
                final_position[i]
                - self.home_position[i]
            )
            for i in range(6)
        )

        if final_error > 1e-6:
            self.get_logger().error(
                f"Trajectory final position is not Home. "
                f"Maximum error: {final_error:.6e} rad"
            )

            return (
                False,
                "Generated trajectory does not "
                "end exactly at Home.",
            )

        # ------------------------------------------------------------------
        # 7. Build JointTrajectory message
        #
        # Still no publication has occurred.
        # ------------------------------------------------------------------

        traj_msg = JointTrajectory()

        traj_msg.joint_names = list(
            self.joint_names
        )

        for point in trajectory:

            traj_point = (
                JointTrajectoryPoint()
            )

            traj_point.positions = list(
                point.positions
            )

            traj_point.velocities = list(
                point.velocities
            )

            seconds = int(
                point.time_from_start
            )

            nanoseconds = int(
                round(
                    (
                        point.time_from_start
                        - seconds
                    )
                    * 1e9
                )
            )

            # Handle possible rounding to exactly 1 second.
            if nanoseconds >= 1_000_000_000:
                seconds += 1
                nanoseconds -= 1_000_000_000

            traj_point.time_from_start.sec = (
                seconds
            )

            traj_point.time_from_start.nanosec = (
                nanoseconds
            )

            traj_msg.points.append(
                traj_point
            )

        # ------------------------------------------------------------------
        # 8. Final pre-publication check
        # ------------------------------------------------------------------

        if self.is_moving:
            return (
                False,
                "Robot became busy before trajectory "
                "publication.",
            )

        self.get_logger().info(
            "=================================================="
        )

        self.get_logger().info(
            "Trajectory validation PASSED."
        )

        self.get_logger().info(
            f"Points: {len(traj_msg.points)}"
        )

        self.get_logger().info(
            f"Duration: {total_time:.3f} s"
        )

        self.get_logger().info(
            "The complete trajectory is considered "
            "safe according to the configured checks."
        )

        self.get_logger().info(
            "Publishing trajectory..."
        )

        self.get_logger().info(
            "=================================================="
        )

        # ------------------------------------------------------------------
        # 9. Execute trajectory
        # ------------------------------------------------------------------

        self.is_moving = True
        self.movement_start_time = time.time()

        try:
            self.trajectory_pub.publish(
                traj_msg
            )

            self.get_logger().info(
                f"Published trajectory with "
                f"{len(traj_msg.points)} points."
            )

            # --------------------------------------------------------------
            # Wait for robot to reach Home.
            #
            # /joint_states continues being processed because we use a
            # MultiThreadedExecutor + ReentrantCallbackGroup.
            # --------------------------------------------------------------

            while self.is_moving:

                elapsed = (
                    time.time()
                    - self.movement_start_time
                )

                # ----------------------------------------------------------
                # Timeout
                # ----------------------------------------------------------

                if elapsed > self.timeout:
                    self.is_moving = False

                    self.get_logger().error(
                        "Go Home timeout exceeded."
                    )

                    return (
                        False,
                        f"Timeout exceeded: "
                        f"movement did not complete "
                        f"within {self.timeout:.1f} seconds.",
                    )

                # ----------------------------------------------------------
                # Communication/state check
                # ----------------------------------------------------------

                if self.current_positions is None:
                    self.is_moving = False

                    self.get_logger().error(
                        "Lost joint states during movement."
                    )

                    return (
                        False,
                        "Lost communication with "
                        "joint state source during movement.",
                    )

                # ----------------------------------------------------------
                # Check Home
                # ----------------------------------------------------------

                if is_at_home(
                    self.current_positions,
                    self.home_position,
                    self.tolerance,
                ):
                    self.is_moving = False

                    self.get_logger().info(
                        "=================================================="
                    )

                    self.get_logger().info(
                        "Go Home completed successfully."
                    )

                    self.get_logger().info(
                        f"Final joint positions: "
                        f"{self.current_positions}"
                    )

                    self.get_logger().info(
                        "=================================================="
                    )

                    return (
                        True,
                        "Robot has reached Home position.",
                    )

                time.sleep(0.05)

        except Exception as exc:
            self.is_moving = False

            self.get_logger().error(
                f"Error during trajectory execution: "
                f"{exc}"
            )

            return (
                False,
                f"Error during trajectory execution: "
                f"{exc}",
            )

    # ======================================================================
    # Shutdown
    # ======================================================================

    def destroy_node(self):
        """Stop movement state before destroying the node."""

        self.is_moving = False

        super().destroy_node()


def main(args=None):
    """Run the Go Home service node."""

    rclpy.init(args=args)

    node = None

    try:
        node = GoHomeService()

        executor = MultiThreadedExecutor(
            num_threads=2
        )

        executor.add_node(node)

        node.get_logger().info(
            "Spinning Go Home service..."
        )

        executor.spin()

    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info(
                "Keyboard interrupt received."
            )

    except Exception as exc:
        if node is not None:
            node.get_logger().fatal(
                f"Fatal error: {exc}"
            )
        else:
            print(
                f"Fatal error while starting node: {exc}"
            )

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

