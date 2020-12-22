import argparse
import sys

from copy import copy

import rospy

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

import intera_interface

from intera_interface import CHECK_VERSION

import modern_robotics as mr
import numpy as np
from geometry_msgs.msg import Point

class Trajectory(object):
    def __init__(self, limb, joint_names):
        self._joint_names = joint_names
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(limb)

        self.duration = 0.0
        self.if_target = False
        self.target = Point()

    def add_point(self, positions, dt):
        self.duration += dt
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(self.duration)

        # Here we add velocities and accelerations
        # v
        if len(self._goal.trajectory.points) > 1:
            point.velocities = 1.0 * (positions - self._goal.trajectory.points[-1].positions) / dt
        else:
            point.velocities = [0, 0, 0, 0, 0, 0, 0]
        # a
        if len(self._goal.trajectory.points) > 2:
            point.accelerations = 1.0 * (point.velocities - self._goal.trajectory.points[-1].velocities) / dt
        else:
            point.accelerations = [0, 0, 0, 0, 0, 0, 0]
        # Code ends

        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = self._joint_names

    def target_callback(self,msg):
        self.if_target = True
        self.target = msg
