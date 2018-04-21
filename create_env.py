#!/usr/bin/env python

import rospy
import math

from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String
from sensor_msgs.msg import Image, JointState, LaserScan, PointCloud2
from rosgraph_msgs.msg import Clock
from kobuki_msgs.msg import MotorPower
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates

angularDiscretizations = [32, 96, 160, 224, 288, 352, 416, 480, 544, 608]

class TurtleBot_ROS():

	rangeValues = []

	def __init__(self, node_name, update_freq):

		rospy.init_node(node_name, anonymous=False)
		rospy.loginfo("To stop the simulation CTRL + C")
		rospy.on_shutdown(self.shutdown)
		self.rate = rospy.Rate(update_freq)
		self.command = Twist()

	# Create several publishers depending upon the type of control we want over the  TurtleBot
	# Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2 - Check for the

	# Defining the Publishers for the several features of the robot to be controlled by the robot

	def create_publishers(self):
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		self.safety = rospy.Publisher('cmd_vel_mux/input/safety_controller', Twist, queue_size=10)

		self.motor_power = rospy.Publisher('/mobile_base/commands/motor_power', MotorPower, queue_size=10)
		self.velocity = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

	# Defining the subscribers

	def create_subscribers(self):
		self.joint_states = rospy.Subscriber('/joint_states', JointState, self.callback_jointstates)
		self.robot_odom = rospy.Subscriber('/odom', Odometry, self.callback_odometry)
		self.laserscan = rospy.Subscriber('/scan', LaserScan, self.callback_laserscan)
		self.gazebo_linkstates = rospy.Subscriber('/gazebo/link_states', LinkStates, self.callback_gazebo_linkstates )

	# Defining Callbacks

	def callback_jointstates(self, data):
		self.jointstate_data = data

	def callback_odometry(self, data):
		self.odom_data = data

	def callback_laserscan(self, data):
		self.laserscan_data = data

		self.rangeValues = []
		for discretization in angularDiscretizations:
			
			val = data.ranges[discretization]

			if math.isnan(val):
				self.rangeValues.append(0.0)
			else:

				minVal = 10.0
				for val in data.ranges[(discretization-32):(discretization+32)]:
					if not math.isnan(val):
						if val<minVal:
							minVal = val

				self.rangeValues.append(minVal)

		print self.rangeValues

	def callback_gazebo_linkstates(self, data):
		self.gz_linkstates = data

	# Simplifying motion commands
	def move(self, forward_vel=0, z_angular=0):
		self.command.linear.x = forward_vel				#Forward motion in m/s
		self.command.linear.y = 0						#Zero since the robot has non-holonomic constraints 
		self.command.angular.z = z_angular				#Angular Speeds in radians/s

		self.cmd_vel.publish(self.command)

	def stop_bot(self):
		self.cmd_vel.publish(Twist())

	def shutdown(self):
			# stop turtlebot
			rospy.loginfo("Stop TurtleBot")
			# a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
			self.stop_bot()
			# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
			rospy.sleep(1)

def main():

	Tbt = TurtleBot_ROS('TurtleBotFLC', 10)
	Tbt.create_publishers()
	Tbt.create_subscribers()

	# -ve angular velocity for Right turn
	# +ve angular velocity for Right turn
	
	while not rospy.is_shutdown():
		Tbt.move(-0.3, 0)



if __name__ == '__main__':
		main()

