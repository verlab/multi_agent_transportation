#!/usr/bin/python

import rospy
from geometry_msgs.msg import Pose

item_publisher = None

def marker_callback(data):
  global item_publisher
  item_publisher.publish( data )

def run():
  global item_publisher

  rospy.init_node("follow_target")

  rospy.Subscriber("/item/current_pose", Pose, marker_callback)

  robot_id = rospy.get_param("~robot_id", 0)
  goal_topic = "/robot_%s/mobile_base/goal" % robot_id

  item_publisher = rospy.Publisher( goal_topic , Pose, queue_size = 1 )

  rospy.spin()

if __name__ == "__main__":
  try:
    run()
  except rospy.ROSInterruptException, e:
    pass
