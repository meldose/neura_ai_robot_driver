cmake_minimum_required(VERSION 3.5)
project(neura_ai_moveit_config VERSION 0.1.0 LANGUAGES CXX)

# if you have extra cmake modules:
# list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
# include(project_info)  # only if cmake/project_info.cmake exists

# find ROS 2 and MoveIt dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning REQUIRED)
# … any other find_package() you need …

# (no libraries or nodes here — this is just a config package)
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


######  ROS 2 env ############
#/usr/bin/env python3


import rclpy #imported rclpy
from rclpy.node import Node # imported Node 
from rclpy.action import ActionClient # imported Action client
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint # imported Jointtrajectory
from sensor_msgs.msg import JointState # imported Jointstate
import time # imported time module
import unittest #imported unittest

# created class FakeJointDriver

class FakeJointDriverTester(Node):
    def __init__(self):
        super().__init__('test_fake_joint_driver_node')
        # Store last received joint states
        self.joint_states = None
        # Subscriber for joint_states
        self.create_subscription(
            JointState,
            'joint_states',
            self.cb_joint_states,
            10
        )
        # Publisher for trajectory commands
        self.pub = self.create_publisher(
            JointTrajectory,
            'joint_trajectory_controller/command',
            10
        )
        # Action client for FollowJointTrajectory
        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            'joint_trajectory_controller/follow_joint_trajectory'
        )

# created function for joint states

    def cb_joint_states(self, msg: JointState):
        self.joint_states = msg

# created class TestFakeJointDriver
class TestFakeJointDriver(unittest.TestCase):

    # created function for setting up the class
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = FakeJointDriverTester()
        # Wait for action server
        available = cls.node.client.wait_for_server(timeout_sec=10.0)
        assert available, 'Action server not available'

    # created function for destroying the node
    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

# function for spining of the node 
    def spin_for(self, duration_sec: float):
        end = time.time() + duration_sec
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)

#created function for test-joint tracjectory command
    def test_joint_trajectory_command(self):
        traj = JointTrajectory()
        # Stamp now
        traj.header.stamp = self.node.get_clock().now().to_msg()
        traj.joint_names = ['JOINT1', 'JOINT2', 'JOINT3']
        # Build a sequence of points
        for i in range(11):
            pt = JointTrajectoryPoint()
            pt.positions = [0.1*i, 0.2*i, 0.3*i]
            pt.time_from_start = rclpy.duration.Duration(seconds=i * 0.1).to_msg()
            traj.points.append(pt)

        # Publish trajectory and spin
        self.node.pub.publish(traj)
        self.spin_for(1.1)

        # Check final positions
        js = self.node.joint_states
        self.assertIsNotNone(js, 'No JointState received')
        self.assertAlmostEqual(js.position[0], 1.0, places=2)
        self.assertAlmostEqual(js.position[1], 2.0, places=2)
        self.assertAlmostEqual(js.position[2], 3.0, places=2)

# created function for test joint trajectory action
    def test_joint_trajectory_action(self):
        # Create and send action goal
        goal_msg = FollowJointTrajectory.Goal()
        traj = goal_msg.trajectory # setting up the trajectory
        traj.header.stamp = self.node.get_clock().now().to_msg() # setting up the header stamp
        traj.joint_names = ['JOINT1', 'JOINT2', 'JOINT3'] # setting the joint names
        for i in range(11):
            pt = JointTrajectoryPoint()
            pt.positions = [0.1*i, 0.2*i, 0.3*i]
            pt.time_from_start = rclpy.duration.Duration(seconds=i * 0.1).to_msg()
            traj.points.append(pt)

        # Send goal and wait
        send_goal_future = self.node.client.send_goal_async(goal_msg) # send the goal future
        rclpy.spin_until_future_complete(self.node, send_goal_future)
        goal_handle = send_goal_future.result()
        self.assertTrue(goal_handle.accepted, 'Goal was not accepted')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)

        # Allow a brief spin to update joint_states
        self.spin_for(0.1)

        # Verify positions
        js = self.node.joint_states
        self.assertIsNotNone(js, 'No JointState received')
        self.assertAlmostEqual(js.position[0], 1.0, places=2)
        self.assertAlmostEqual(js.position[1], 2.0, places=2)
        self.assertAlmostEqual(js.position[2], 3.0, places=2)

# calling the main function

if __name__ == '__main__':
    # Initialize and run tests
    rclpy.init()
    unittest.main()
    rclpy.shutdown()
