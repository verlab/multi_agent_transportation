#!/usr/bin/python

import rospy

def run():
  rospy.init_node("item_tracker")

  model_name = rospy.get_param("~model_name", None)
  simulation = rospy.get_param("~simulation", True)
  item_topic = "current_pose"

  if simulation:
    from tracking_system.tracker import SimulationTracker

    model_name = model_name if model_name is not None else "item"

    tracker = SimulationTracker( model_name, item_topic )
    tracker.run()
  else:
    from tracking_system.real_tracker import RealTracker

    item_marker_id = rospy.get_param("~item_marker_id", 0)
    RealTracker(item_marker_id, item_topic).run()

  rospy.spin()

if __name__ == "__main__":
  try:
    run()
  except rospy.ROSInterruptException, e:
    pass
