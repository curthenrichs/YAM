#!/usr/bin/env python

import rospy

from std_msgs.msg import Bool, String
from sensor_msgs.msg import Range, BatteryState
from yam_msgs.msg import Ultrasonics, RobotState
from yam_msgs.msg import CartesianDrive, DifferentialDrive


class NavigationFirmwareDriver:
	
	def __init__(self):
		self._hw_wallbanger_pub = rospy.Publisher("rosserial/ai",Bool,queue_size=5)
		self._hw_cartesian_pub = rospy.Publisher("rosserial/cd",CartesianDrive,queue_size=5)
		self._hw_differential_pub = rospy.Publisher("rosserial/dd",DifferentialDrive,queue_size=5)
		
		self._left_range_pub = rospy.Publisher("range/left",Range,queue_size=5)
		self._center_range_pub = rospy.Publisher("range/center",Range,queue_size=5)
		self._right_range_pub = rospy.Publisher("range/right",Range,queue_size=5)
		
		self._battery_pub = rospy.Publisher("battery_state",BatteryState,queue_size=5)
		self._watchdog_tripped_pub = rospy.Publisher("navigation_firmware/watchdog_tripped",Bool,queue_size=5)
		self._robot_state_pub = rospy.Publisher("navigation_firmware/state",String,queue_size=5)
		self._get_wallbanger_state_pub = rospy.Publisher("navigation_firmware/get_wallbanger_state",Bool,queue_size=5)
		
		self._cartesian_drive_sub = rospy.Subscriber("navigation_firmware/cartesian_drive",CartesianDrive,self._cartesian_drive_cb)
		self._differential_drive_sub = rospy.Subscriber("navigation_firmware/differential_drive",DifferentialDrive,self._differential_drive_cb)
		
		self._set_wallbanger_state_sub = rospy.Subscriber("navigation_firmware/set_wallbanger_state",Bool,self._set_wallbanger_state_cb)
		self._hw_ultrasonics_sub = rospy.Subscriber("rosserial/us",Ultrasonics,self._hw_ultrasonics_cb)
		self._hw_robot_state_sub = rospy.Subscriber("rosserial/rs",RobotState,self._hw_robot_state_cb)

	def _cartesian_drive_cb(self, msg):
		self._hw_cartesian_pub.publish(msg)
		
	def _differential_drive_cb(self, msg):
		self._hw_differential_pub.publish(msg)

	def _set_wallbanger_state_cb(self, msg):
		self._hw_wallbanger_pub.publish(msg)

	def _hw_ultrasonics_cb(self, msg):
		
		leftMsg = Range()
		leftMsg.header.stamp = rospy.Time.now()
		leftMsg.header.frame_id = "left_ultrasonic"
		leftMsg.radiation_type = Range.ULTRASOUND
		leftMsg.field_of_view = msg.field_of_view
		leftMsg.min_range = msg.min_range
		leftMsg.max_range = msg.max_range
		leftMsg.range = msg.left_range
		
		self._left_range_pub.publish(leftMsg)
		
		centerMsg = Range()
		centerMsg.header.stamp = rospy.Time.now()
		centerMsg.header.frame_id = "center_ultrasonic"
		centerMsg.radiation_type = Range.ULTRASOUND
		centerMsg.field_of_view = msg.field_of_view
		centerMsg.min_range = msg.min_range
		centerMsg.max_range = msg.max_range
		centerMsg.range = msg.center_range
		
		self._center_range_pub.publish(centerMsg)
		
		rightMsg = Range()
		rightMsg.header.stamp = rospy.Time.now()
		rightMsg.header.frame_id = "right_ultrasonic"
		rightMsg.radiation_type = Range.ULTRASOUND
		rightMsg.field_of_view = msg.field_of_view
		rightMsg.min_range = msg.min_range
		rightMsg.max_range = msg.max_range
		rightMsg.range = msg.right_range
		
		self._right_range_pub.publish(rightMsg)
		
	def _hw_robot_state_cb(self, msg):
		self._get_wallbanger_state_pub.publish(msg.in_autonomous_mode)
		
		batteryMsg = BatteryState()
		
		if msg.robot_power_state == RobotState.ESTOP_ACTIVE:
			batteryMsg.voltage = float('nan')
			batteryMsg.percentage = float('nan')
		else:
			batteryMsg.voltage = msg.voltage # volts
			batteryMsg.percentage = max(0, min(msg.voltage / 12.0, 1)) # volts / volts-nominal
		
		batteryMsg.current = float('nan')
		batteryMsg.charge = float('nan')
		batteryMsg.capacity = float('nan')
		batteryMsg.design_capacity = 14.0 #AH (2x batteries)
		batteryMsg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
		batteryMsg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
		batteryMsg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
		batteryMsg.present = True
		
		self._battery_pub.publish(batteryMsg)
		
		self._watchdog_tripped_pub.publish(msg.watchdog_tripped)
		
		strMsg = String()
		if msg.robot_power_state == RobotState.POWER_GOOD:
			strMsg.data = "power-good"
		elif msg.robot_power_state == RobotState.POWER_LOW:
			strMsg.data = "power-low"
		elif msg.robot_power_state == RobotState.POWER_EMERGENCY:
			strMsg.data = "power-emergency"
		elif msg.robot_power_state == RobotState.ESTOP_ACTIVE:
			strMsg.data = "estop-active"
			
		self._robot_state_pub.publish(strMsg)


if __name__ == "__main__":
	rospy.init_node("navigation_firmware_driver")

	node = NavigationFirmwareDriver()
	
	rospy.spin()
