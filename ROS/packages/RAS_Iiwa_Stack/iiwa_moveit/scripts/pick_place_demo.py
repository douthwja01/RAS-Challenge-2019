#!/usr/bin/env python
import sys
import re
import rospy
import moveit_commander
from std_msgs.msg import String
from moveit_msgs.msg import PickupAction, Grasp, CollisionObject, RobotState
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from math import pi, radians, degrees
from actionlib import SimpleActionClient
from tf.transformations import *

class PickPlace():

	def __init__(self):
		# Initializing the node
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('iiwa_pick_place', anonymous=False)
		move_group = "manipulator"

		# Instantiating a Robot Commander
		self._robot = moveit_commander.RobotCommander()

		# Instantiating a Scene
		self._scene = moveit_commander.PlanningSceneInterface()

		# Instantiating a Move Group
		self._move_group = moveit_commander.MoveGroupCommander(move_group)

		# Publisher for Kuka API
		self._kuka_api_pub = rospy.Publisher('/moveit_iiwa', String, queue_size=10)

	# Moves robots to joint goal
	def move_to_joint_goal(self, joint_positions):
		joint_goal = self._move_group.get_current_joint_values()
		joint_goal = joint_positions
		print(joint_goal)
		self._move_group.go(joint_goal, wait=True)

		self._move_group.stop()

	# Moves the robot to a pose
	def move_to_pose(self):
		pose_goal = Pose()

		# Setting up RPY so that the end effector is always facing downwards
		q = quaternion_from_euler(0, pi, 0)

		pose_goal.orientation.x = q[0]
		pose_goal.orientation.y = q[1]
		pose_goal.orientation.z = q[2]
		pose_goal.orientation.w = q[3]
		pose_goal.position.x = .0
		pose_goal.position.y = .0
		pose_goal.position.z = .0

		print(pose_goal)
		prev_pose = self._move_group.get_current_joint_values()
		self._move_group.set_pose_target(pose_goal)

		plan = self._move_group.go(wait=True)
		new_pose = self._move_group.get_current_joint_values()


		self._move_group.stop()

		self._move_group.clear_pose_targets()

	# Moves the robot to save position, defined in MoveIt! Config
	def move_to_saved_position(self, name):
		self._move_group.set_named_target(name)
		rate = rospy.Rate(10)		
		plan1 = self._move_group.go()

		rospy.sleep(5)
		msg_to_send = "setPosition "
		joint_positions_string = ""
		joint_positions = self._move_group.get_current_joint_values()
		for x in joint_positions:
			joint_positions_string += str(math.degrees(x)) + " "
		msg_to_send += joint_positions_string
		msg_to_send.strip()
		print(msg_to_send)
		rospy.sleep(1)

		self._kuka_api_pub.publish(msg_to_send)
		print(self._kuka_api_pub)
		rate.sleep()

	def add_table(self, name):
		p = PoseStamped()
		p.header.frame_id =	self._robot.get_planning_frame()
		p.header.stamp = rospy.Time.now()
		p.pose.position.x =	.7
		p.pose.position.y =	.0
		p.pose.position.z =	.2
		q =	quaternion_from_euler(0.0,	0.0, numpy.deg2rad(90.0))
		p.pose.orientation = Quaternion(*q)
		self._scene.add_box(name, p, (1.5,0.8,0.03))

	# Gets the joint state from the robot
	# And sets the initial state of the robot in Rviz
	def set_robots_initial_state(self):
		# Subscriber to /JointPosition topic
		self._join_state_sub = rospy.Subscriber('/JointPosition', String, self.joint_position_received)
		
	# Callback function, called when the data is received by the _joint_state_sub subscriber
	def joint_position_received(self, data):
		# Data received by subscriber is in .data attribute
		joint_data = data.data
		# Data cleaning:
		## Removig all characters past ']'
		## Removing '[', ']', ',' characters
		## Spliting the string into array (String contains joint values separated by spaces)
		## Converting joint values from degrees (use by robot's API) to radians (used by Rviz)
		joint_data = joint_data[:joint_data.index(']')+1]
		joint_data = re.sub('[\[\],]', '', joint_data).split()
		joint_data = [math.radians(float(i)) for i in joint_data]		

		# Moves the robot in Rviz to extracted position
		self._move_group.go(joint_data)

		# Unregisteres from the topic
		self._join_state_sub.unregister()

if __name__ == '__main__':
	pick_place = PickPlace()
	rospy.sleep(2)
	#pick_place.set_robots_initial_state()
	#rospy.sleep(2)
	pick_place.move_to_pose()

	# Necessary to keep the node running
	rospy.spin()
