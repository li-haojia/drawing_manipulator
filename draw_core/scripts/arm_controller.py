#!/usr/bin/env python
import sys
import rospy

# moveit stuff
import moveit_commander

# msg stuff
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String


def controller(): 
    moveit_commander.roscpp_initialize(sys.argv)     # TODO: what for?
    rospy.init_node('drawing_control', anonymous = True)

    manipulator = moveit_commander.RobotCommander()
    group_name = "manipulator_i5"
    group = moveit_commander.MoveGroupCommander(group_name)

    # initial settings
    reference = String()
    reference = "world"
    group.set_pose_reference_frame(reference)
    group.allow_replanning(True)
    group.set_max_velocity_scaling_factor(0.4)      # TODO: What for?
    group.set_max_acceleration_scaling_factor(0.3)
    group.set_goal_orientation_tolerance(0.1)
    group.set_goal_position_tolerance(0.1)
    group.set_planning_time(3.0)

    # get current state
    current_state = geometry_msgs.msg.Pose()
    # current_state = manipulator.get_current_state()
    current_state = group.get_current_pose().pose

    # set target state
    target_state = geometry_msgs.msg.Pose()
    target_state.position.x = current_state.position.x
    target_state.position.y = current_state.position.y
    target_state.position.z = current_state.position.z - 0.3
    target_state.orientation.x = current_state.orientation.x
    target_state.orientation.y = current_state.orientation.y
    target_state.orientation.z = current_state.orientation.z
    target_state.orientation.w = current_state.orientation.w

    group.set_pose_target(target_state)

    # plan and execute
    plan = group.go(wait = True)
    group.stop()
    group.clear_pose_targets()

if __name__=="__main__":
    controller()



