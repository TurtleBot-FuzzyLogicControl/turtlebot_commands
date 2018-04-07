#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String
from sensor_msgs.msg import Image, JointState, LaserScan, PointCloud2
from rosgraph_msgs.msg import Clock
from kobuki_msgs.msg import MotorPower
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates

class TurtleBot_ROS():

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
		self.switch = rospy.Publisher('cmd_vel_mux/input/switch', Twist, queue_size=10)
		self.safety = rospy.Publisher('cmd_vel_mux/input/safety_controller', Twist, queue_size=10)
		self.teleop = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)

		self.mot_power = rospy.Publisher('/mobile_base/commands/motor_power', MotorPower, queue_size=10)
		self.velo = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

	# Defining the subscribers

	def create_subscribers(self):
		self.image_raw = rospy.Subscriber('camera/depth/image_raw', Image, self.callback_im_raw)
		self.image_depth = rospy.Subscriber('camera/depth/points', PointCloud2, self.callback_im_depth)
		self.joint_states = rospy.Subscriber('/joint_states', JointState, self.callback_jointstates)
		self.robot_odom = rospy.Subscriber('/odom', Odometry, self.callback_odometry)
		self.laserscan = rospy.Subscriber('/scan', LaserScan, self.callback_laserscan)
		self.gazebo_linkstates = rospy.Subscriber('/gazebo/link_states', LinkStates, self.callback_gazebo_linkstates )

	# Defining Callbacks
	def callback_im_raw(self, data):
		self.raw_image_data = data

	def callback_im_depth(self, data):
		self.depth_image_data = data

	def callback_jointstates(self, data):
		self.jointstate_data = data

	def callback_odometry(self, data):
		self.odom_data = data

	def callback_laserscan(self, data):
		self.laserscan_data = data

	def callback_gazebo_linkstates(self, data):
		self.gz_linkstates = data

	# Simplifying motion commands
	def move(self, x_speed=0, y_speed=0, z_angular=0):
		self.command.linear.x = x_speed
		self.command.linear.y = y_speed					#Linear Speeds in m/s
		self.command.angular.z = z_angular				#Angular Speeds in radians/s

	def do_motion(self):
		while not rospy.is_shutdown():
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

	Tbt = TurtleBot_ROS('Test', 10)
	Tbt.create_publishers()
	Tbt.create_subscribers()
	Tbt.move(-0.2, -0.2, 0.1)
	Tbt.do_motion()

if __name__ == '__main__':
		main()























