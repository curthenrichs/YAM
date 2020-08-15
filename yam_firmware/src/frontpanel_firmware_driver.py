#!/usr/bin/env python

import rospy


class FrontpanelFirmwareDriver:
	
	def __init__(self):
		pass
		
		
if __name__ == "__main__":
	rospy.init_node("frontpanel_firmware_driver")

	node = FrontpanelFirmwareDriver()
	
	rospy.spin()
