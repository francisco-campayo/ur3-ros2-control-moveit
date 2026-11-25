"""Integration tests for the Go Home service node.

These tests use launch_testing to verify the complete behavior of the
go_home_service node in a simulated ROS2 environment.
"""

import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory


class TestGoHomeService(unittest.TestCase):
    """Integration tests for the Go Home service."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 for the test class."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2 after all tests."""
        rclpy.shutdown()

    def setUp(self):
        """Create a test node for each test."""
        self.node = Node('test_go_home_client')
        self.joint_states_received = []
        self.lock = threading.Lock()

        # Target joint positions to publish continuously
        self.target_joint_positions = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]

        # Create client for the service
        self.client = self.node.create_client(SetBool, 'go_home')

        # Create publisher to simulate joint states
        self.joint_pub = self.node.create_publisher(
            JointState,
            '/joint_states',
            10,
        )

        # Create timer to publish joint states continuously
        self.joint_state_timer = self.node.create_timer(
            0.1,
            self.publish_joint_states_timer_callback,
        )

        # Create subscription to monitor trajectory commands
        self.trajectory_received = False
        self.node.create_subscription(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            self.trajectory_callback,
            10,
        )

    def tearDown(self):
        """Clean up after each test."""
        self.node.destroy_node()

    def publish_joint_states_timer_callback(self):
        """Publish target joint states continuously."""
        msg = JointState()
        msg.name = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]
        msg.position = self.target_joint_positions
        self.joint_pub.publish(msg)

    def trajectory_callback(self, msg):
        """Track if trajectory commands are received."""
        with self.lock:
            self.trajectory_received = True

    def set_target_positions(self, positions):
        """Set the target joint positions to publish."""
        self.target_joint_positions = positions

    def spin_for(self, seconds):
        """Spin the test node for a while.

        time.sleep() does NOT execute callbacks: the joint state timer would
        never fire and go_home_service would never receive /joint_states.
        """
        end_time = time.monotonic() + seconds
        while time.monotonic() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def call_service(self, timeout=20.0):
        """Call the go_home service and wait for response."""
        max_retries = 15
        retry_count = 0
        while not self.client.wait_for_service(timeout_sec=1.0):
            retry_count += 1
            if retry_count > max_retries:
                raise RuntimeError('Service not available after 15 retries')

        request = SetBool.Request()
        request.data = True

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)

        if future.done():
            return future.result()
        else:
            raise TimeoutError('Service call timed out')

    def test_service_available(self):
        """Test that the go_home service is available."""
        # Wait for service to be available with longer timeout
        available = self.client.wait_for_service(timeout_sec=30.0)
        self.assertTrue(
            available,
            'go_home service should be available'
        )

    def test_already_at_home(self):
        """Test response when robot is already at home position."""
        # Set home position
        home = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        self.set_target_positions(home)

        # Wait for node to process
        self.spin_for(1.0)

        # Call service
        response = self.call_service(timeout=10.0)

        self.assertTrue(response.success)
        self.assertIn('already at home', response.message.lower())

    def test_invalid_request(self):
        """Test response to invalid request (data=false)."""
        # Set some position
        self.set_target_positions([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
        self.spin_for(1.0)

        # Call service with invalid request
        if not self.client.wait_for_service(timeout_sec=10.0):
            self.fail('Service not available')

        request = SetBool.Request()
        request.data = False

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)

        self.assertTrue(future.done())
        response = future.result()
        self.assertFalse(response.success)
        self.assertIn('invalid', response.message.lower())

    def test_trajectory_published(self):
        """Test that trajectory is published when moving to home."""
        # Set non-home position
        not_home = [1.0, -1.0, 1.0, -1.0, -1.0, 1.0]
        self.set_target_positions(not_home)
        self.spin_for(1.0)

        # Reset trajectory flag
        with self.lock:
            self.trajectory_received = False

        # Call service
        try:
            self.call_service(timeout=20.0)
        except TimeoutError:
            pass  # Timeout is acceptable for this test

        # Check that trajectory was published
        self.spin_for(1.0)
        with self.lock:
            self.assertTrue(
                self.trajectory_received,
                'Trajectory should be published when moving to home'
            )


def generate_test_description():
    """Generate launch description for the test."""
    go_home_node = launch_ros.actions.Node(
        package='my_ur_control',
        executable='go_home_service',
        name='go_home_service',
        output='screen',
    )

    return launch.LaunchDescription([
        go_home_node,
        launch_testing.actions.ReadyToTest(),
    ])


if __name__ == '__main__':
    launch_testing.main()
