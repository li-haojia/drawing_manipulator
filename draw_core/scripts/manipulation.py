#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import modern_robotics as mr
import numpy as np
import intera_interface
from geometry_msgs.msg import Point
from std_msgs.msg import String


class PositionControl(object):

    def __init__(self):
        # Initialize ROS node
        rospy.init_node("manipulation", anonymous=True)
        self.point_pub = rospy.Publisher("/me495/current_position", Point, queue_size = 10)

        # Communicate with perception node
        self.tag_number = 0
        rospy.Subscriber('/me495/target_position', Point, self.callback_point)
        # Wait for command
        rospy.Subscriber("/me495/command", String, self.callback_command)

        # Enable the Sawyer
        rs = intera_interface.RobotEnable()
        rs.enable()
        # Set the right arm and velocity ratio
        self.mylimb = intera_interface.Limb("right")
        self.mylimb.set_joint_position_speed(0.2)
        # Dictionary for store target position in joint space
        self.waypoints = self.mylimb.joint_angles()

        # Command sent to perception node
        self.command = Point()
        self.command.x = 0
        self.command.y = 0
        self.command.z = 0

        # IK parameter
        self.Slist = np.array([[0, 0,  1,      0,     0,       0],
                               [0, 1,  0, -0.317,     0,   0.081],
                               [1, 0,  0,      0, 0.317, -0.1925],
                               [0, 1,  0, -0.317,     0,   0.481],
                               [1, 0,  0,      0, 0.317,  -0.024],
                               [0, 1,  0, -0.317,     0,   0.881]]).T
        self.M = np.array([[ 0,  0, 1, 1.01475],
                           [-1,  0, 0,  0.1603],
                           [ 0, -1, 0,   0.317],
                           [ 0,  0, 0,       1]])
        self.eomg = 0.01
        self.ev = 0.001

        # Turn head for taking photo
        head = intera_interface.Head()
        head.set_pan(angle=-np.pi, speed=0.3, timeout=10, active_cancellation=True)


    def callback_command(self, string):
        if string.data == "Localize Board":
            print("Start Localizing")
            self.first_move()



    def callback_point(self, point):
        self.point = point
        rospy.sleep(0.1)
        # print("recieving", self.point)

        if not (point.x == 0 and point.y == 0 and point.z == 0):
            self.T[0][3] = point.x
            self.T[1][3] = point.y
            self.T[2][3] = point.z

            # self.thetalist = [self.mylimb.joint_angle(joint) for joint in self.mylimb.joint_names()]
            # self.thetalist = self.thetalist[0:6]
            self.thetalist = [0.0, -0.55, 0.0, 1.89, 0.0, -1.37]
            self.move_to_each(self.thetalist)
            rospy.sleep(0.5)
            self.tag_number = self.tag_number + 1

        rospy.sleep(1)
        if self.tag_number == 4:
            self.finish()


    def first_move(self):
        # Turn head for showing the edge drawing
        head = intera_interface.Head()
        head.set_pan(angle=0.0, speed=0.3, timeout=10, active_cancellation=True)

        # First observe all the AR_tags
        # Target point
        self.point = Point()
        self.point.x = 0.700
        self.point.y = 0.1603
        self.point.z = 0.650
        # This is the initial configuration of right_hand_camera in base frame
        # The desired end-effector configuration
        self.T = np.array([[ 0,  0,  1, self.point.x],
		                   [-1,  0,  0, self.point.y],
		                   [ 0, -1,  0, self.point.z],
		                   [ 0,  0,  0,            1]])
        # thetalist0; current joint angles
        self.thetalist = [0.00722578, -1.13799861, -0.01079448, 1.7510739, 0.00573321, 0.95772104]
        # Move to initial position
        self.move_to_high(self.thetalist)


    def move_to_high(self, thetalist0):
        # Solve IK
        self.thetalist, success = mr.IKinSpace(self.Slist, self.M, self.T, thetalist0, self.eomg, self.ev)
        self.IK_validation(self.thetalist)

        # Nove to the initial position
        self.waypoints['right_j0'] = self.thetalist[0]
        self.waypoints['right_j1'] = self.thetalist[1]
        self.waypoints['right_j2'] = self.thetalist[2]
        self.waypoints['right_j3'] = self.thetalist[3]
        self.waypoints['right_j4'] = self.thetalist[4]
        self.waypoints['right_j5'] = self.thetalist[5]
        self.waypoints['right_j6'] = 0.0
        self.mylimb.move_to_joint_positions(self.waypoints, timeout = 20.0, threshold = 0.05)

        # Publish the command to perception node
        rospy.sleep(1)
        # (-1, 0, 0) for perceiving all four tags
        self.command.x = -1
        self.point_pub.publish(self.command)


    def move_to_each(self, thetalist0):
        # Solve IK
        self.thetalist, success = mr.IKinSpace(self.Slist, self.M, self.T, thetalist0, self.eomg, self.ev)
        self.IK_validation(self.thetalist)

        # Nove to the initial position
        self.waypoints['right_j0'] = self.thetalist[0]
        self.waypoints['right_j1'] = self.thetalist[1]
        self.waypoints['right_j2'] = self.thetalist[2]
        self.waypoints['right_j3'] = self.thetalist[3]
        self.waypoints['right_j4'] = self.thetalist[4]
        self.waypoints['right_j5'] = self.thetalist[5]
        self.waypoints['right_j6'] = 0.0
        self.mylimb.move_to_joint_positions(self.waypoints, timeout = 20.0, threshold = 0.05)

        # Publish the command to perception node
        # (0, i, 0) for perceiving the tag i
        rospy.sleep(0.5)
        self.command.x = 0
        self.command.y = self.tag_number
        self.point_pub.publish(self.command)
        rospy.sleep(1)


    def finish(self):
        self.command.x = 1
        self.point_pub.publish(self.command)


    def IK_validation(self, thetalist):
        # Stop the drawing if the IK solution is not valid
        for i in range(len(thetalist)):
            if abs(thetalist[i]) > np.pi:
                print("Bad IK solution!!!!!! Stop the robot.")
                rospy.signal_shutdown("Invalid IK solution")



if __name__ == "__main__":
    pc = PositionControl()
    while not rospy.is_shutdown():
        rospy.spin()
