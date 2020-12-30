#!/usr/bin/env python
import sys
import rospy
import time

# moveit stuff
import moveit_commander

# msg stuff
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

class Arm_Contrl:
    def __init__(self, P_A, P_B):
        moveit_commander.roscpp_initialize(sys.argv)     
        rospy.init_node('drawing_control', anonymous = True)

        self.manipulator = moveit_commander.RobotCommander()
        self.group_name = "manipulator_i5"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)

        # for real world settings
        self.height = 0.15

        # initial settings
        reference = String()
        reference = "world"
        self.group.set_pose_reference_frame(reference)
        self.group.allow_replanning(True)
        self.group.set_max_velocity_scaling_factor(0.4)      
        self.group.set_max_acceleration_scaling_factor(0.3)
        self.group.set_goal_orientation_tolerance(0.02)
        self.group.set_goal_position_tolerance(0.02)
        self.group.set_planning_time(6.0)

        # set target A and B
        self.A = P_A
        self.B = P_B

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
        


    def controller(self): 
        self.move(self.A)
        
        # move done
        # get current state    
        current_state = geometry_msgs.msg.Pose()
        current_state = self.group.get_current_pose().pose   
        print(current_state)                          

        # set target state
        goal_state = geometry_msgs.msg.Pose()
        goal_state.position.x = current_state.position.x
        goal_state.position.y = current_state.position.y
        goal_state.position.z = current_state.position.z - self.height
        goal_state.orientation.x = current_state.orientation.x
        goal_state.orientation.y = current_state.orientation.y
        goal_state.orientation.z = current_state.orientation.z
        goal_state.orientation.w = current_state.orientation.w                  

        self.move(goal_state)
        # end move done

        self.move(self.B)

        # move up 
        # get current state    
        current_state = geometry_msgs.msg.Pose()
        current_state = self.group.get_current_pose().pose                             

        # set target state
        goal_state = geometry_msgs.msg.Pose()
        goal_state.position.x = current_state.position.x
        goal_state.position.y = current_state.position.y
        goal_state.position.z = current_state.position.z + self.height
        goal_state.orientation.x = current_state.orientation.x
        goal_state.orientation.y = current_state.orientation.y
        goal_state.orientation.z = current_state.orientation.z
        goal_state.orientation.w = current_state.orientation.w                  

        self.move(goal_state)
        # end move up


if __name__=="__main__":
    # test goal
    P_A = geometry_msgs.msg.Pose()
    P_A.position.x = -0.22
    P_A.position.y = -0.01
    P_A.position.z = 1.26
    P_A.orientation.x = 0.69
    P_A.orientation.y = -0.00
    P_A.orientation.z = -0.00
    P_A.orientation.w = 0.71

    P_B = geometry_msgs.msg.Pose()
    P_B.position.x = -0.42
    P_B.position.y = -0.41
    P_B.position.z = 1.26
    P_B.orientation.x = 0.69
    P_B.orientation.y = -0.00
    P_B.orientation.z = -0.00
    P_B.orientation.w = 0.71 
    
    # end test
    control = Arm_Contrl(P_A, P_B)
    control.controller()



