#!/usr/bin/env python

import sys
import rospy
import roslaunch

rospy.init_node('build_roslib')
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

delete_node = roslaunch.core.Node('yam_firmware','delete_roslib.sh', args='{0}'.format(sys.argv[1]))
std_lib_build_node = roslaunch.core.Node('rosserial_arduino','make_libraries.py', args='{0}'.format(sys.argv[1]))
yam_lib_build_node = roslaunch.core.Node('rosserial_client','make_library.py', args='{0} {1}'.format(sys.argv[1], 'yam_msgs'))


launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

rospy.loginfo("started")

rospy.loginfo("deleting")
process = launch.launch(delete_node)
while process.is_alive():
    rospy.loginfo(".")
    rospy.sleep(1)
process.stop()

rospy.loginfo("making arduino standard libraries")
process = launch.launch(std_lib_build_node)
while process.is_alive():
    rospy.loginfo(".")
    rospy.sleep(1)
process.stop()


#rospy.loginfo("making yam_msgs library")
#process = launch.launch(std_lib_build_node)
#while process.is_alive():
#    rospy.loginfo(".")
#    rospy.sleep(1)
#process.stop()

rospy.loginfo("done!")

#launch.shutdown()
