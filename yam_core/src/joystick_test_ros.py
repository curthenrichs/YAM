#!/usr/bin/env python

import rospy

from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from yam_msgs.msg import CartesianDrive


X_AXIS = 0
Y_AXIS = 1
A_BUTTON = 0


class JoystickMapToRobotNode:
	
	def __init__(self):
		self._ai_state = False
		self._prev_button_state = 0
		
		self._cartesian_drive_pub = rospy.Publisher("navigation_firmware/cartesian_drive",CartesianDrive,queue_size=5)
		self._set_wallbanger_state_pub = rospy.Publisher("navigation_firmware/set_wallbanger_state",Bool,queue_size=5)
	
		self._joystick_sub = rospy.Subscriber("joy",Joy,self._joystick_cb)
		
		rospy.sleep(10) # arbitrary sleep
		
		self._set_wallbanger_state_pub.publish(Bool(self._ai_state))
		
	def _joystick_cb(self, msg):
		print "Joystick message received"
		
		button_state = msg.buttons[A_BUTTON]
		
		if button_state == 1 and self._prev_button_state == 0:
			self._ai_state = not self._ai_state
			self._set_wallbanger_state_pub.publish(Bool(self._ai_state))
		self._prev_button_state = button_state
		
		if not self._ai_state:
			drive = CartesianDrive()
			drive.x = -1 * self.__clamp(100 * msg.axes[X_AXIS])
			drive.y = self.__clamp(100 * msg.axes[Y_AXIS])
			
			self._cartesian_drive_pub.publish(drive)
			
	def __clamp(self,v):
		if v > 100:
			return 100
		elif v < -100:
			return -100
		else:
			return v
		
	
if __name__ == "__main__":
	rospy.init_node("joystick_test_ros")
	
	print "Creating mapping node"
	
	node = JoystickMapToRobotNode()
	
	print "Created mapping node"
	
	rospy.spin()
