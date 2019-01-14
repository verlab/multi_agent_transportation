#!/usr/bin/python

import sys
sys.path.append("/home/ramonmelo/Dropbox/Ramon_Projects/ramon-mestrado/")

import rospy
from system_utils import launcher_lib
from nautilus_executor.utils import *
from nautilus_executor.manager_multi import ManagerHungarianMulti
import pickle

def run():
  rospy.init_node("system_manager")

  map_path = rospy.get_param("~map_path")

  environment = create_environment( map_path )
  environment_encoded = pickle.dumps( environment )

  rospy.set_param("/environment", environment_encoded)
  rospy.set_param("/environment_ready", False)
  rospy.set_param("/allocation_done", False)

  rospy.loginfo("Initializing simulation")

  while not rospy.get_param("/environment_ready") and not rospy.is_shutdown():
    rospy.loginfo("wait for simulation")
    rospy.sleep(1)

  rospy.loginfo("starting planning")

  manager = ManagerHungarianMulti()
  allocation_list = manager.execute( environment )

  allocation_encoded = pickle.dumps(allocation_list)

  rospy.set_param("/current_phase", 0)
  rospy.set_param("/allocation", allocation_encoded)
  rospy.set_param("/allocation_done", True)

  rospy.loginfo("wait be done")
  rospy.spin()

  # for item in object_list:
  #   object_id = item["id"]
  #   start = item["start"]
  #   end = item["end"]

  #   rospy.set_param("/done", False)
  #   rospy.set_param("/allocation_done", False)
  #   rospy.set_param("/current_phase", 0)

  #   # Update maps obstacles
  #   local_map = maps.copy()

  #   other_object_list = filter(lambda o: o['id'] != object_id, object_list)

  #   for other_item in other_object_list:
  #     # request for positions
  #     other_object_id = other_item['id']
  #     other_object = other_item['start']

  #     service_name = "/item_%i/current_pose_scalled" % other_object_id

  #     rospy.wait_for_service(service_name)
  #     res = rospy.ServiceProxy(service_name, CurrentPoseScalled)()

  #     x, y = res.x, res.y

  #     other_object.position = ( x, y, other_object.z )

  #     local_map.add_obstacle( other_object )

  #   # Update robot positions
  #   for robot in robot_list:
  #     service_name = "/robot_%i/current_pose_scalled" % robot.id

  #     rospy.wait_for_service(service_name)
  #     res = rospy.ServiceProxy(service_name, CurrentPoseScalled)()

  #     x, y = res.x, res.y

  #     robot.position = ( x, y, robot.z )

  #   workspace = {
  #     'maps': local_map,
  #     'initial_state': start,
  #     'goal_state': end,
  #     'robot_descriptors': robot_list
  #   }

  #   workspace_encoded = encode_data( workspace )
  #   rospy.set_param("/workspace", workspace_encoded)

  #   launcher_lib.launch( "nautilus_manager", "planning.launch" )

  #   rospy.loginfo("planning done")

  #   while not rospy.get_param("/done") and not rospy.is_shutdown():
  #     rospy.loginfo("wait be done")
  #     rospy.sleep(5)

  # rospy.loginfo("all done")

  # rospy.spin()

if __name__ == "__main__":
  try:
    run()
  except rospy.ROSInterruptException:
    pass
