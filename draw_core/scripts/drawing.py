#!/usr/bin/env python
import argparse
import sys

from copy import copy

import rospy

import actionlib

import modern_robotics as mr
import numpy as np

from geometry_msgs.msg import Point
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)


import intera_interface

from intera_interface import CHECK_VERSION

from trajectory import Trajectory

from std_msgs.msg import String, Int8MultiArray

class DrawingControl(object):
    def __init__(self):
        #first enable the robot
        self.limb, self.limb_interface=self.enable_robot()

        self.start_flag=False
        self.target=Point()
        self.plot_points=[]

        self.state_target = False
        self.state_trajectory = False
        rospy.Subscriber("/me495/location", Point, self.callback_target)
        rospy.Subscriber("/me495/trajectory", Int8MultiArray, self.callback_traj)

        rospy.Subscriber('/me495/command',String,self.callback_start)
        #plot_points = [(0.575,0.1603),(0.575,0.2603),(-1,-1),(0.575,0.1603)]
        #Once all set, ask controller whether to start

        self.start_pub = rospy.Publisher('/me495/result', String, queue_size = 1)

        while not rospy.is_shutdown():
            if self.state_target == True and self.state_trajectory == True:
                self.start_pub.publish("All Set")
                self.state_target = False


    def callback_target(self, point):
        self.target = point
        self.state_target = True
        print("Received Target location")

    def callback_traj(self, int8MultiArray):
        self.trajectory = int8MultiArray.data
        self.trajectory = np.array(self.trajectory).reshape(len(self.trajectory)/2,2)
        self.state_trajectory = True
        print("Received Drawing Trajectory")


    def callback_start(self, string):
        if string.data == "Start Drawing":
            print("Begin To Draw")
            #define the view_traj which move robot from current joint_positions to view position
            #_drawing_control.view_trajectory(limb,limb_interface)
            #define the traj from view point to target point
            self.view_target_trajectory(self.limb,self.limb_interface)
            #define the test circle trajectory
            #plot_circle_trajectory(limb,limb_interface)
            #define the face trajectory
            self.plot_face_trajectory(self.limb,self.limb_interface)


    def enable_robot(self):
        rp = intera_interface.RobotParams()
        valid_limbs = rp.get_limb_names()
        if not valid_limbs:
            rp.log_message(("Cannot detect any limb parameters on this robot. "
              "Exiting."), "ERROR")
            return

        # arg_fmt = argparse.RawDescriptionHelpFormatter
        # parser = argparse.ArgumentParser(formatter_class=arg_fmt,
        #                                  description=main.__doc__)
        # parser.add_argument(
        #     '-l', '--limb', choices=valid_limbs, default=valid_limbs[0],
        #     help='send joint trajectory to which limb'
        # )
        #
        # args = parser.parse_args(rospy.myargv()[1:])
        # limb = args.limb
        limb = "right"

        rospy.init_node("drawing".format(limb))
        #print("Getting robot state... ")
        rs = intera_interface.RobotEnable(CHECK_VERSION)
        #print("Enabling robot... ")
        #rs.enable()
        limb_interface = intera_interface.Limb(limb)

        return limb, limb_interface

    def get_parameters_for_IK(self):
        Slist = np.array([[0, 0,  1,      0,     0,       0],
                          [0, 1,  0, -0.317,     0,   0.081],
                          [1, 0,  0,      0, 0.317, -0.1925],
                          [0, 1,  0, -0.317,     0,   0.481],
                          [1, 0,  0,      0, 0.317,  -0.024],
                          [0, 1,  0, -0.317,     0,   0.881],
                          [1, 0,  0,      0, 0.317, -0.1603]]).T
        M = np.array([[0, 0, 1, 1.01475], [-1, 0, 0, 0.1603], [0, -1, 0, 0.317], [0, 0, 0, 1]])
        # This is the configuration of the center point.

        eomg = 0.01
        ev = 0.001
        return Slist,M,eomg,ev

    def view_trajectory(self,limb,limb_interface):
        view_traj = Trajectory(limb, limb_interface.joint_names())
        rospy.on_shutdown(view_traj.stop)
        # get Current Joint Positions first
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        view_traj.add_point(current_angles, 0.0)
        #joint-angles when sawyer at view point
        thetalist0 = [0, -np.pi / 2.0, 0, np.pi / 2, 0, np.pi / 2, 0]
        n_sec = 10.0
        view_traj.add_point(thetalist0, n_sec)
        view_traj.start()
        view_traj.wait(view_traj.duration)
        #now we have moved to the view point
        #publish a message that robot have arrived at view point
        # overview_point = Point()
        # overview_point.z = 1.0
        # point_pub = rospy.Publisher('current_position', Point, queue_size = 1)
        # point_pub.publish(overview_point)



    def view_target_trajectory(self,limb,limb_interface):
        view_target_traj=Trajectory(limb, limb_interface.joint_names())
        rospy.on_shutdown(view_target_traj.stop)
        Slist, M, eomg, ev = self.get_parameters_for_IK()
        T = np.array([[ 0, -1,  0,  0.575],
                      [-1,  0,  0, 0.1603],
                      [ 0,  0, -1,  0.317],
                      [ 0,  0,  0,      1]])

        # Wait for the target_position
        self.view_trajectory(limb,limb_interface)
        # target = Point()
        # target.x = 0.575
        # target.y = 0.1603
        # target.z = 0.0
        #default step: add current__angles at beginning of every trajectory
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        view_target_traj.add_point(current_angles, 0.0)
        #set the target point
        T[0][3] = self.target.x
        T[1][3] = self.target.y
        T[2][3] = self.target.z + 0.2
        #solve the IK
        thetalist0, success = mr.IKinSpace(Slist, M, T, current_angles, eomg, ev)
        n_sec = 10.0
        view_target_traj.add_point(thetalist0, n_sec)
        view_target_traj.start()
        view_target_traj.wait(view_target_traj.duration)


    def plot_face_trajectory(self,limb, limb_interface):
        Slist, M, eomg, ev = self.get_parameters_for_IK()
        n_sec = 0.5
        n_sec_for_hang=5
        z_touch=0.160
        z_hang=0.210
        x_cutoff=-0.20
        y_cutoff=-0.17
        T = np.array([[0, -1, 0, 0.575],
                      [-1, 0, 0, 0.1603],
                      [0, 0, -1, 0.317],
                      [0, 0, 0, 1]])

        #plot_points=self.plot_points

        i=0
        marker_hanup=True
        line_traj = Trajectory(limb, limb_interface.joint_names())
        rospy.on_shutdown(line_traj.stop)

        #default step, add current_angles as initial thetalist0
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        thetalist0=current_angles
        line_traj.add_point(current_angles, 0.0)
        #start drawing position
        thetalist0 = [0, -np.pi / 3.0, 0, np.pi / 2, 0, np.pi / 6, 0]
        line_traj.add_point(thetalist0, n_sec_for_hang)

        plot_points =self.trajectory

        print("Solving IK")
        thetalist0 = [-0.66, -0.91, 0.04, 2.21, -0.11, 0.28, -0.45]
        while i < len(plot_points):
            if marker_hanup==True:
                #move right above the start point
                T[0][3] = plot_points[i][0]*0.005 + self.target.x + x_cutoff
                T[1][3] = plot_points[i][1]*0.005 + self.target.y + y_cutoff
                thetalist0, success = mr.IKinSpace(Slist, M, T, thetalist0, eomg, ev)
                # print("Bad IK Solution point: ", T[0][3], T[1][3], T[2][3], i,plot_points[i][0], plot_points[i][1])
                # print("Joint angles: ", thetalist0)
                self.IK_validation(thetalist0)
                if i==0:
                    print("fisrt angles",thetalist0)
                line_traj.add_point(thetalist0, n_sec_for_hang)
                #drop down marker
                T[2][3] = self.target.z + z_touch
                thetalist0, success = mr.IKinSpace(Slist, M, T, thetalist0, eomg, ev)
                # print("Bad IK Solution point: ", T[0][3], T[1][3], T[2][3], i,plot_points[i][0], plot_points[i][1])
                # print("Joint angles: ", thetalist0)
                self.IK_validation(thetalist0)
                line_traj.add_point(thetalist0, n_sec_for_hang)

                marker_hanup=False
                i+=1
            else:
                if plot_points[i][0]==-1 and plot_points[i][1]==-1:
                    # lift up the marker
                    T[2][3] = self.target.z + z_hang
                    thetalist0, success = mr.IKinSpace(Slist, M, T, thetalist0, eomg, ev)
                    # print("Bad IK Solution point: ", T[0][3], T[1][3], T[2][3], i,plot_points[i][0], plot_points[i][1])
                    # print("Joint angles: ", thetalist0)
                    self.IK_validation(thetalist0)
                    line_traj.add_point(thetalist0, n_sec_for_hang)
                    marker_hanup=True
                else:
                    T[0][3] = plot_points[i][0]*0.005 +self.target.x +x_cutoff
                    T[1][3] = plot_points[i][1]*0.005 +self.target.y +y_cutoff
                    thetalist0, success = mr.IKinSpace(Slist, M, T, thetalist0, eomg, ev)
                    # print("Bad IK Solution point: ", T[0][3], T[1][3], T[2][3], i,plot_points[i][0], plot_points[i][1])
                    # print("Joint angles: ", thetalist0)
                    self.IK_validation(thetalist0)
                    line_traj.add_point(thetalist0, n_sec)
                i+=1
        print("Finishing Solving IK")

        #start to draw
        line_traj.start()
        line_traj.wait(line_traj.duration)
        print("Finish Drawing !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")


    def IK_validation(self, thetalist):
        # Stop the drawing if the IK solution is not valid
        for i in range(len(thetalist)):
            if abs(thetalist[i]) > np.pi:
                print("Bad IK solution!!!!!! Stop the robot.")
                rospy.signal_shutdown("Invalid IK solution")
                # pass



if __name__ == "__main__":
    drawing_control = DrawingControl()
