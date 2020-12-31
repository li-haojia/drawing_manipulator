#!/usr/bin/env python
import sys
import rospy
import time
import copy
import math

# moveit stuff
import moveit_commander

# msg stuff
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

class Arm_Contrl():
    def __init__(self):
        self.manipulator = moveit_commander.RobotCommander()
        self.group_name = "manipulator_i5"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)

        # for real world settings
        self.height = 0.15

        # initial settings
        reference = String()
        reference = "base_link"
        self.group.set_pose_reference_frame(reference)
        self.group.allow_replanning(True)
        self.group.set_max_velocity_scaling_factor(1.0)      
        self.group.set_max_acceleration_scaling_factor(1.0)
        self.group.set_goal_orientation_tolerance(0.1)
        self.group.set_goal_position_tolerance(0.001)
        self.group.set_planning_time(6.0)

        # for drawing settings
        self.line_gap = 0.01

    def get_current_pos(self):
        return self.group.get_current_pose().pose

    def go_home(self):
        # goal = geometry_msgs.msg.Pose()
        # goal = self.group.get_current_pose().pose
        joint_positions = [0.09833356546924307, -0.13036996652071509, -1.665223463667989, 0.010214026325672795, -1.604895446066772, 0.0984022748028983]
        self.group.set_joint_value_target(joint_positions)
        # goal.position.x = -0.2
        # goal.position.y = -0.2
        # goal.position.z = 1.05
        # goal.orientation.x = 0.0
        # goal.orientation.y = 0.0
        # goal.orientation.z = 0.0
        # goal.orientation.w = 1.0

        # self.group.set_pose_target(goal)
        plan = self.group.go(wait = True)
        time.sleep(1.0)
        self.group.clear_pose_targets()
        print("I am home now")


    def move(self, goal_state):
        # get current state
        current_state = geometry_msgs.msg.Pose()
        current_state = self.group.get_current_pose().pose

        self.group.set_pose_target(goal_state)

        # plan and execute
        plan = self.group.go(wait = True)
        time.sleep(5.0)
        # self.group.stop()
        self.group.clear_pose_targets()
        print("achieve move goal")

    def draw_line(self, end_point):
        # get current state
        current_state = geometry_msgs.msg.Pose()
        current_state = self.group.get_current_pose().pose

        line_points = []
        line_points.append(current_state)
        # delta_x = end_point.position.x - current_state.position.x
        # delta_y = end_point.position.y - current_state.position.y
        # dist = math.sqrt(delta_x * delta_x + delta_y * delta_y)
        # # cos_theta = (dist * dist + delta_x * delta_x - delta_y * delta_y) / 2 * dist * delta_x
        # # sin_theta = math.sqrt(1 - cos_theta * cos_theta)
        # num_points = int(dist / self.line_gap)
        # print("dist", dist)
        # print("num_points", num_points)

        temp_point = geometry_msgs.msg.Pose()
        temp_point.position.x = current_state.position.x
        temp_point.position.y = current_state.position.y
        temp_point.position.z = current_state.position.z
        temp_point.orientation.x = current_state.orientation.x
        temp_point.orientation.y = current_state.orientation.y
        temp_point.orientation.z = current_state.orientation.z
        temp_point.orientation.w = current_state.orientation.w    

        # for i in range(num_points):
        #     temp_point.position.x += delta_x / num_points
        #     temp_point.position.y += delta_y / num_points
        #     line_points.append(temp_point)
        temp_point.position.x = end_point.position.x
        temp_point.position.y = end_point.position.y
        line_points.append(temp_point)
        
        # draw the line
        (line_traj, fraction) = self.group.compute_cartesian_path(line_points, 0.01, 0,avoid_collisions= False)
        self.group.execute(line_traj, wait=True)
        print("draw the line!")
        time.sleep(1.0)
                


    # def controller(self): 
    #     # move done
    #     # get current state    
    #     current_state = geometry_msgs.msg.Pose()
    #     current_state = self.group.get_current_pose().pose                          

    #     # set target state
    #     goal_state = geometry_msgs.msg.Pose()
    #     goal_state.position.x = current_state.position.x
    #     goal_state.position.y = current_state.position.y
    #     goal_state.position.z = current_state.position.z - self.height
    #     goal_state.orientation.x = current_state.orientation.x
    #     goal_state.orientation.y = current_state.orientation.y
    #     goal_state.orientation.z = current_state.orientation.z
    #     goal_state.orientation.w = current_state.orientation.w                  

    #     self.move(goal_state) 
    #     # end move done

    #     # set plan
    #     waypoints = []
    #     current_state = geometry_msgs.msg.Pose()
    #     current_state = self.group.get_current_pose().pose     
    #     waypoints.append(current_state)
    #     waypoints += self.waypoints
    #     (self.plan, fraction) = group.compute_cartesian_path(waypoints, 0.1, 0,0)

    #     self.group.execute(self.plan, wait=True)
    #     print("achieve plan goal")
    #     time.sleep(1.0)

    #     # move up 
    #     # get current state    
    #     current_state = geometry_msgs.msg.Pose()
    #     current_state = self.group.get_current_pose().pose                             

    #     # set target state
    #     goal_state = geometry_msgs.msg.Pose()
    #     goal_state.position.x = current_state.position.x
    #     goal_state.position.y = current_state.position.y
    #     goal_state.position.z = current_state.position.z + self.height
    #     goal_state.orientation.x = current_state.orientation.x
    #     goal_state.orientation.y = current_state.orientation.y
    #     goal_state.orientation.z = current_state.orientation.z
    #     goal_state.orientation.w = current_state.orientation.w                  

    #     self.move(goal_state)
    #     # end move up


if __name__=="__main__":
    moveit_commander.roscpp_initialize(sys.argv)     
    rospy.init_node('drawing_control', anonymous = True)
    
    
    # test plan  
    manipulator = moveit_commander.RobotCommander()
    group_name = "manipulator_i5"
    group = moveit_commander.MoveGroupCommander(group_name)

    waypoints = []

    p = group.get_current_pose().pose
    j = group.get_current_joint_values()
    print("current pose", p)
    print("current_joint_values", j)
    coef = -1.0

    p.position.x += 0.05 * coef
    p.position.y += 0.05 * coef
    waypoints.append(copy.deepcopy(p))

    p.position.x += 0.05 * coef
    p.position.y += 0.05 * coef
    waypoints.append(copy.deepcopy(p))

    p.position.x += 0.05 * coef
    p.position.y += 0.05 * coef
    waypoints.append(copy.deepcopy(p))

    p.position.x += 0.05 * coef
    p.position.y += 0.05 * coef
    waypoints.append(copy.deepcopy(p))

    p.position.x += 0.05 * coef
    p.position.y += 0.05 * coef
    waypoints.append(copy.deepcopy(p))
    # end test

    # test line drawer
    p0 = group.get_current_pose().pose
    p1 = copy.deepcopy(p0)
    coef = -1.0
    p1.position.x += 0.25 * coef
    p1.position.y += 0.25 * coef
    # end test


    control = Arm_Contrl()
    # control.draw_line(p1)
    control.go_home()



