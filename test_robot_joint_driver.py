

######  ROS 2 env ############

#/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import time
import unittest

class RobotJointDriverTester(Node):
    def __init__(self):
        super().__init__('test_robot_joint_driver_node')
        # Last received joint states
        self.joint_states = None
        # Subscribe to joint_states
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

    def cb_joint_states(self, msg: JointState):
        self.joint_states = msg

class TestRobotJointDriver(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = RobotJointDriverTester()
        # Wait for action server
        available = cls.node.client.wait_for_server(timeout_sec=10.0)
        assert available, 'Action server not available'

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_for(self, duration_sec: float):
        end = time.time() + duration_sec
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def test_joint_trajectory_command(self):
        traj = JointTrajectory()
        # Stamp with current time
        traj.header.stamp = self.node.get_clock().now().to_msg()
        traj.joint_names = [
            'joint1','joint2','joint3',
            'joint4','joint5','joint6','joint7'
        ]
        # Build trajectory points
        for i in range(11):
            pt = JointTrajectoryPoint()
            pt.positions = [
                0.0*i, 0.0*i, 0.157*i,
                0.0*i, 0.0*i, 0.0*i, 0.157*i
            ]
            pt.time_from_start = rclpy.duration.Duration(seconds=i * 0.1).to_msg()
            traj.points.append(pt)

        # Publish and spin
        self.node.pub.publish(traj)
        self.spin_for(1.1)

        js = self.node.joint_states
        self.assertIsNotNone(js, 'No JointState received')
        # Verify final positions
        for idx in range(7):
            self.assertAlmostEqual(js.position[idx], 1.57, places=2)

    def test_joint_trajectory_action(self):
        goal_msg = FollowJointTrajectory.Goal()
        traj = goal_msg.trajectory
        traj.header.stamp = self.node.get_clock().now().to_msg()
        traj.joint_names = [
            'joint1','joint2','joint3',
            'joint4','joint5','joint6','joint7'
        ]
        for i in range(11):
            pt = JointTrajectoryPoint()
            pt.positions = [
                0.0*i, 0.0*i, 0.157*i,
                0.0*i, 0.0*i, 0.0*i, 0.157*i
            ]
            pt.time_from_start = rclpy.duration.Duration(seconds=i * 0.1).to_msg()
            traj.points.append(pt)

        # Send goal and wait
        send_goal_future = self.node.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, send_goal_future)
        goal_handle = send_goal_future.result()
        self.assertTrue(goal_handle.accepted, 'Goal was not accepted')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)

        # Brief spin to update joint_states
        self.spin_for(0.1)

        js = self.node.joint_states
        self.assertIsNotNone(js, 'No JointState received')
        for idx in range(7):
            self.assertAlmostEqual(js.position[idx], 1.57, places=2)

if __name__ == '__main__':
    rclpy.init()
    unittest.main()
    rclpy.shutdown()
