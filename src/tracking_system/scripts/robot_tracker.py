#!/usr/bin/python

import rospy

def run():
  rospy.init_node("robot_tracker")

  simulation = rospy.get_param("~simulation", True)
  robot_topic = "current_pose"

  if simulation:
    from tracking_system.tracker import SimulationTracker

    robot_prefix = rospy.get_param("~robot_prefix", "robot_prefix")
    # robot_prefix = "%s" % robot_prefix

    SimulationTracker(robot_prefix, robot_topic).run()

  else:
    from tracking_system.real_tracker import RealTracker

    robot_marker_id = rospy.get_param("~robot_marker_id", 0)
    RealTracker(robot_marker_id, robot_topic).run()

  rospy.spin()

if __name__ == "__main__":
  try:
    run()
  except rospy.ROSInterruptException, e:
    pass
