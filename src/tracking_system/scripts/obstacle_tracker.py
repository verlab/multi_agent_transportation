#!/usr/bin/python

import rospy
import simplejson as json
from tracking_system.real_tracker import RealTracker

from std_msgs.msg import String

marker_list = {}

def marker_callback( valid_markers ):
  global marker_list

  temp_list = {}
  for marker in valid_markers:
    pos = marker.pose.pose.position

    temp_list[ marker.id ] = {
      "x": pos.x,
      "y": pos.y
    }

  marker_list = temp_list

def run():
  global marker_list

  rospy.init_node("obstacle_tracker")

  RealTracker(marker_id_list = range(20, 92), callback = marker_callback).run()

  info_publisher = rospy.Publisher("/obstacle_info", String, queue_size = 10)

  rate = rospy.Rate(1)
  while not rospy.is_shutdown():
    data = json.dumps( marker_list )

    info = String()
    info.data = data

    info_publisher.publish( info )

    rate.sleep()

if __name__ == "__main__":
  try:
    run()
  except rospy.ROSInterruptException, e:
    pass
