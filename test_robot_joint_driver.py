#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction)
from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint)
import rospy
import rostest
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import unittest

class TestRobotJointDriver(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_robot_joint_driver_node')

    def setUp(self):
        rospy.Subscriber('joint_states',
                         JointState, self.cb_joint_states, queue_size=10)
        self.joint_states = rospy.wait_for_message(
            'joint_states', JointState, timeout=10.0)
        
        self.pub = rospy.Publisher(
            'joint_trajectory_controller/command',
            JointTrajectory,
            queue_size=10)
        
        self.client = actionlib.SimpleActionClient(
            'joint_trajectory_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction)

        self.client.wait_for_server(timeout=rospy.Duration(10.0))
        
    def cb_joint_states(self, msg):
        self.joint_states = msg

    def test_joint_trajectory_command(self):
        traj = JointTrajectory()
        traj.header.stamp = rospy.Time(0)
        traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        for i in range(11):
            point = JointTrajectoryPoint()
            point.positions = [0.0*i, 0.0*i, 0.157*i, 0.0*i, 0.0*i, 0.0*i, 0.157*i]
            point.time_from_start = rospy.Duration(i*0.1)
            traj.points.append(point)

        self.pub.publish(traj)
        rospy.sleep(1.1)
        self.assertAlmostEqual(self.joint_states.position[0], 1.57)
        self.assertAlmostEqual(self.joint_states.position[1], 1.57)
        self.assertAlmostEqual(self.joint_states.position[2], 1.57)
        self.assertAlmostEqual(self.joint_states.position[3], 1.57) 
        self.assertAlmostEqual(self.joint_states.position[4], 1.57) 
        self.assertAlmostEqual(self.joint_states.position[5], 1.57) 
        self.assertAlmostEqual(self.joint_states.position[6], 1.57) 
        
if __name__ == '__main__':
    rostest.rosrun('robot_joint_driver',
                   'test_robot_joint_driver',
                   TestRobotJointDriver)
