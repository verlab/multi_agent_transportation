#!/usr/bin/python

import rospy
from tracking_system.real_tracker import RealTracker

def run():
  rospy.init_node("item_target_tracker")

  item_target_topic = "/item_target/current_pose"

  item_target_marker_id = rospy.get_param("~item_target_marker_id", 0)
  RealTracker(item_target_marker_id, item_target_topic).run()

  rospy.spin()

if __name__ == "__main__":
  try:
    run()
  except rospy.ROSInterruptException, e:
    pass
