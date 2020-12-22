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
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)     
        rospy.init_node('drawing_control', anonymous = True)

        self.manipulator = moveit_commander.RobotCommander()
        self.group_name = "manipulator_i5"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)

        # initial settings
        reference = String()
        reference = "world"
        self.group.set_pose_reference_frame(reference)
        self.group.allow_replanning(True)
        self.group.set_max_velocity_scaling_factor(0.4)      # TODO: What for?
        self.group.set_max_acceleration_scaling_factor(0.3)
        self.group.set_goal_orientation_tolerance(0.1)
        self.group.set_goal_position_tolerance(0.1)
        self.group.set_planning_time(3.0)

        # set target A and B
        # self.A = A
        # self.B = B

    def move(self, goal_state):
        # get current state
        current_state = geometry_msgs.msg.Pose()
        current_state = self.group.get_current_pose().pose

        self.group.set_pose_target(goal_state)

        # plan and execute
        plan = self.group.go(wait = True)
        self.group.stop()
        self.group.clear_pose_targets()
        time.sleep(1.0)


    def controller(self): 
        # move done
        # get current state    
        current_state = geometry_msgs.msg.Pose()
        current_state = self.group.get_current_pose().pose   
        # print(current_state)                          

        # set target state
        goal_state = geometry_msgs.msg.Pose()
        goal_state.position.x = current_state.position.x
        goal_state.position.y = current_state.position.y
        goal_state.position.z = current_state.position.z + 0.1
        goal_state.orientation.x = current_state.orientation.x
        goal_state.orientation.y = current_state.orientation.y
        goal_state.orientation.z = current_state.orientation.z
        goal_state.orientation.w = current_state.orientation.w                  

        self.move(goal_state)
        # end move done

        # self.move(self.A)

        # self.move(self.B)

        # move up 
        # get current state    
        current_state = geometry_msgs.msg.Pose()
        current_state = self.group.get_current_pose().pose                             

        # set target state
        goal_state = geometry_msgs.msg.Pose()
        goal_state.position.x = current_state.position.x
        goal_state.position.y = current_state.position.y
        goal_state.position.z = current_state.position.z - 0.1
        goal_state.orientation.x = current_state.orientation.x
        goal_state.orientation.y = current_state.orientation.y
        goal_state.orientation.z = current_state.orientation.z
        goal_state.orientation.w = current_state.orientation.w                  

        self.move(goal_state)
        # end move up


if __name__=="__main__":
    control = Arm_Contrl()
    control.controller()



