#!/usr/bin/env python

# teleoperation of a simulated 2D BARC carii
import rospy
from barc.msg import ECU
from geometry_msgs.msg import Twist

# initialize the state
motor = 0
servo = 0

def keypress_callback(msg):
	# interpret different keypresses
	global motor, servo

	linear = msg.linear.x
	angular = msg.angular.z

	# map from vanilla teleop_twist values to our own here

	motor = linear
	servo = angular

class RateRelay(object):
	def __init__ (self, dt=0.05):
		self.sub=rospy.Subscriber('cmd_vel', Twist, keypress_callback)
		self.pub = rospy.Publisher('ecu', ECU, queue_size=10)
		self.rate = rospy.Rate(1.0/dt) # 20 Hz

	def _publish_ecu(self):
		# assemble the message to publish
		# acceleration command from keyboard
		acc_cmd = motor

		# steering angle command from keyboard
		d_f_cmd = servo

		ecu_out = ECU(acc_cmd, d_f_cmd)

		self.pub.publish(ecu_out)

	def run(self):
		while not rospy.is_shutdown():
			self._publish_ecu()
			self.rate.sleep()

def main():
	relay = RateRelay()
	relay.run()
	rospy.spin()

if __name__ =='__main__':
	rospy.init_node('barc_teleop')
	main()