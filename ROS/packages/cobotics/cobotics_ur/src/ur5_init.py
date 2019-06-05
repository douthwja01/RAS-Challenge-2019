#!/usr/bin/env python
#import time
#import roslib; roslib.load_manifest('ur_driver')
#import rospy
#import actionlib
#from control_msgs.msg import *
#from trajectory_msgs.msg import *

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

print "============ Starting tutorial setup"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)

print "Continuing.."
# Instantiate a RobotCommander object. This object is an interface to the robot as a whole
robot = moveit_commander.RobotCommander()
# Instantiate a PlanningSceneInterface object
scene = moveit_commander.PlanningSceneInterface()
# Instantiate a MoveGroupCommander object
group = moveit_commander.MoveGroupCommander("manipulator")
# We create this DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory,
				    queue_size=10)
print "============ Waiting for RVIZ..."
rospy.sleep(10)
print "============ Starting tutorial "

# Getting basic information
print "============ Reference frame: %s" % group.get_planning_frame()
print "============ Reference frame: %s" % group.get_end_effector_link()
print "============ Robot Groups:"
print robot.get_group_names()
print "============ Printing robot state"
print robot.get_current_state()
print "============"


print "============ Generating plan 1"
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 0
pose_target.position.x = -0.0
pose_target.position.y = -0.5 # y horizontal
pose_target.position.z = 0.0

#group.set_planning_time(10)
group.set_pose_target(pose_target)

#Now, we call the planner to compute the plan and visualize it if successful 
plan1 = group.plan()

print "============ Waiting while RVIZ displays plan1..."
rospy.sleep(5)

print "============ Visualizing plan1"
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan1)
display_trajectory_publisher.publish(display_trajectory);

print "============ Waiting while plan1 is visualized (again)..."
rospy.sleep(5)

# Try to move the robot to the goal
group.go(wait=True)

print "End (indigo error below)"
print "========================"


