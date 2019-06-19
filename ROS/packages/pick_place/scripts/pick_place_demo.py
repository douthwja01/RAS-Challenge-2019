#!/usr/bin/env python
import sys
import re
import rospy
import moveit_commander
from std_msgs.msg import String
from moveit_msgs.msg import PickupAction, Grasp, CollisionObject, RobotState, MoveGroupActionFeedback
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from math import pi, radians, degrees
from actionlib import SimpleActionClient
from tf.transformations import *
import readchar
import subprocess as sp

class PickPlace():

	def __init__(self):
		try:
			# Initializing the node
			moveit_commander.roscpp_initialize(sys.argv)
			rospy.init_node('iiwa_pick_place', anonymous=False)
			move_group = "manipulator"
			sim = rospy.get_param("/simulation")

			# Instantiating a Robot Commander
			self._robot = moveit_commander.RobotCommander()

			# Instantiating a Scene
			self._scene = moveit_commander.PlanningSceneInterface()

			# Instantiating a Move Group
			self._move_group = moveit_commander.MoveGroupCommander(move_group)

			# Publisher for Kuka API
			self._kuka_api_pub = rospy.Publisher('/moveit_iiwa', String, queue_size=10)

			# Poses 
			self._pose_over_tools = self.create_pose(0.999, -0.000, -0.001, 0.045, 0.365, 0.164, 0.623)
			self._pose_to_tools = self.create_pose(0.999, -0.001, -0.001, 0.045, 0.365, 0.155, 0.525)
			self._pose_place_away = self.create_pose(0.999, 0.000, -0.001, 0.045, 0.687, 0.143, 0.385)
			self._pose_place_table = self.create_pose(0.999, 0.000, -0.001, 0.045, 0.687, 0.143, 0.285)
			self._pose_home = self.create_pose(-0.000, 0.000, 0.000, 1.000, 0.000, 0.000, 1.306)

		except:
			print("An error occured. Make sure you are running MoveIt simulation first!")
			print('Exiting application')
			rospy.sleep(2)
			#sys.exit()

	# Moves robots to joint goal
	def move_to_joint_goal(self, joint_positions):
		joint_goal = self._move_group.get_current_joint_values()
		joint_goal = joint_positions
		print(joint_goal)
		self._move_group.go(joint_goal, wait=True)

		self._move_group.stop()

	# Returns the Pose object created from arguments passed in
	def create_pose(self, *args):
		print(args)
		pose = Pose()

		pose.orientation.x = args[0]
		pose.orientation.y = args[1]
		pose.orientation.z = args[2]
		pose.orientation.w = args[3]
		pose.position.x = args[4]
		pose.position.y = args[5]
		pose.position.z = args[6]

		return pose

	# Moves the robot to a pose
	def move_to_pose(self):
		## PICK AND PLACE SEQUENCE
		## 1. Move over the tools
		## 2. Open grippers
		## 3. Move to pick up a tool
		## 4. Close grippers
		## 5. Place the tool away
		## 6. Open grippers
		## 7. Return to home

		print('Moving over the tools')
		self._move_group.set_pose_target(self._pose_over_tools)
		self._move_group.go(wait=True)
		self._move_group.stop()
		self._move_group.clear_pose_targets()
		self.move_robot()
		print('I like to move it, move it!')

		print('Openning grippers')
		#self.open_grippers()
		#self._move_group.go(wait=True)
		#self._move_group.stop()
		#self._move_group.clear_pose_targets()
		print('Open up my eager eyes!')

		print('Picking up a tool')
		self._move_group.set_pose_target(self._pose_to_tools)
		self._move_group.go(wait=True)
		self._move_group.stop()
		self._move_group.clear_pose_targets()
		self.move_robot()
		print('I got to move like Jagger!')

		print('Closing grippers')
		#self.close_grippers()
		#self._move_group.go(wait=True)
		#self._move_group.stop()
		#self._move_group.clear_pose_targets()
		print('Grippers closed')

		print('Moving over the tools')
		self._move_group.set_pose_target(self._pose_over_tools)
		self._move_group.go(wait=True)
		self._move_group.stop()
		self._move_group.clear_pose_targets()
		self.move_robot()
		print('I like to move it, move it!')

		print('Placing away the tool')
		self._move_group.set_pose_target(self._pose_place_away)
		self._move_group.go(wait=True)
		self._move_group.stop()
		self._move_group.clear_pose_targets()
		self.move_robot()
		print('Tool placed away')

		print('Placing away the tool')
		self._move_group.set_pose_target(self._pose_place_table)
		self._move_group.go(wait=True)
		self._move_group.stop()
		self._move_group.clear_pose_targets()
		self.move_robot()
		print('Tool placed away')

		print('Openning grippers')
		#self.open_grippers()
		#self._move_group.go(wait=True)
		#self._move_group.stop()
		#self._move_group.clear_pose_targets()
		print('Grippers opened')

		print('Returning home')
		self._move_group.set_pose_target(self._pose_home)
		self._move_group.go(wait=True)
		self._move_group.stop()
		self._move_group.clear_pose_targets()
		self.move_robot()
		print('Nothing feels like home anymore')

	# Moves the robot (in simulation) back to home position
	def move_to_home(self):
		print('Returning home')
		self._move_group.set_pose_target(self._pose_home)
		self._move_group.go(wait=True)
		self._move_group.stop()
		self._move_group.clear_pose_targets()
		self.move_robot()
		print('Nothing feels like home anymore')

	# Sends the message to Kuka API asking the robot to open grippers
	def open_grippers()
		open_grippers_msg = 'OpenGripper'
		self._kuka_api_pub.publish(open_grippers_msg)
	
	# Sends the message to Kuka API asking the robot to close grippers
	def close_grippers()
		close_grippers_msg = 'CloseGripper'
		self._kuka_api_pub.publish(close_grippers_msg)

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

	# Sends the command to Kuka API to move the actual robot
	def move_robot(self):
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
		pass

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

def display_menu(pick_place):
	print('\n\n\n')
	print('*'*40)
	print('*'*40)
	print('**'+' '*36+'**')
	print('**'+' '*36+'**')
	print('**'+' '*12+'RAS HACKATHON'+' '*11+'**')
	print('**'+' '*36+'**')
	print('**'+' '*36+'**')
	print('*'*40)
	print('*'*40)
	print('1. Show `Pick and Place` Demonstration')
	print('2. Reset robot to Home Position')
	print('3. Exit')
	print('What do you want to do?')
	try:
		choice = int(readchar.readchar())
	except:
		print('Only numbers allowed')
	
	while (choice > 5):
		print('Please choose one of the valid options [1-5]')
		try:
			choice = int(readchar.readchar())
		except:
			print('Only numbers allowed')

	if choice == 3:
		print('Closing an application')
		sys.exit()
	elif choice == 2:
		pick_place.move_to_home()
		pass
	elif choice == 1:
		pick_place.move_to_pose()
		pass

if __name__ == '__main__':
	pick_place = PickPlace()
	rospy.sleep(2)
	display_menu(pick_place)
	#pick_place.set_robots_initial_state()
	#rospy.sleep(2)
	#pick_place.move_to_pose()

	# Necessary to keep the node running
	#rospy.spin()
