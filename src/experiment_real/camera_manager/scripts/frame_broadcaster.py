#!/usr/bin/env python
import roslib
import time

import rospy
import tf

if __name__ == '__main__':
	rospy.init_node('frame_broadcaster')

	br = tf.TransformBroadcaster()
	rate = rospy.Rate(1)

	target_frame = rospy.get_param("~target_frame", "usb_cam_frame")

	# Camera position
	# Translation
	x = rospy.get_param("~x",0)
	y = rospy.get_param("~y",0)
	z = rospy.get_param("~z",0)
	# Pose quaternion
	qm = rospy.get_param("~qm",0)
	qx = rospy.get_param("~qx",0)
	qy = rospy.get_param("~qy",0)
	qz = rospy.get_param("~qz",1)

	# while not rospy.is_shutdown():
	while True:
		br.sendTransform((x,y,z), (qm, qx, qy, qz), rospy.Time.now(), target_frame, "world")
		time.sleep(1.5)
		# rate.sleep()
