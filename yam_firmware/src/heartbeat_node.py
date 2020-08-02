#!/usr/bin/env python

import rospy

from std_msgs.msg import Bool


DEFAULT_HEARTBEAT_TIME = 0.5


class HeartbeatNode:
	
	def __init__(self, heartbeat):
		self._heartbeat_enabled = True

		self._hw_heartbeat_pub = rospy.Publisher("rosserial/hb",Bool,queue_size=5)
		
		rospy.Timer(rospy.Duration(heartbeat),self._heartbeat_cb)
		
	def _heartbeat_cb(self, event):
		self._hw_heartbeat_pub.publish(self._heartbeat_enabled)
		

if __name__ == "__main__":
	rospy.init_node("navigation_firmware_driver")
	
	heartbeat_time = rospy.get_param("heartbeat_time",DEFAULT_HEARTBEAT_TIME)
	
	node = HeartbeatNode(heartbeat_time)
	
	rospy.spin()
