#!/usr/bin/python

import rospy

from system_utils import launcher_lib

from system_utils.workspace_lib import RealWorkscapeLib
from system_utils.encode_decode_lib import encode_data

from nautilus_planner.view_data import viewer

def run():
  rospy.init_node("system_real_manager")

  robot_id_list = rospy.get_param("/robot_id_list", [])

  # Init read workspace

  workspace = RealWorkscapeLib( robot_id_list = robot_id_list )
  workspace.run()

  rospy.sleep(10)

  # Create environment

  rospy.loginfo( "Creating environment" )

  environment = workspace.create_environment_configuration()

  rospy.loginfo( environment )

  environment_encoded = encode_data( environment )

  rospy.loginfo( "Setting configurations" )

  rospy.set_param("/environment", environment_encoded)
  rospy.set_param("/allocation_done", False)
  rospy.set_param("/current_phase", 0)

  rospy.loginfo( "Launch planner" )

  launcher_lib.launch( "nautilus_manager", "planning_real.launch" )

  rospy.spin()

if __name__ == "__main__":
  try:
    run()
  except rospy.ROSInterruptException:
    pass
