#!/usr/bin/python

import sys
sys.path.append("/home/ramonmelo/Dropbox/Ramon_Projects/ramon-mestrado/")

import rospy

from simulation_manager.srv import ObjectState, ObjectStateResponse
from std_srvs.srv import Empty, EmptyResponse
from system_utils import launcher_lib

from nautilus_lib.models.robot import Robot
from nautilus_lib.models.thing import Thing
from nautilus_lib.models.workspace import Workspace

import pickle

class WorldManager(object):
  """docstring for WorldManager"""

  def __init__(self):
    self.scale = 0.34 # 0.173

  def run(self):
    rospy.loginfo("Creating world...")

    rospy.set_param("/world_scale", self.scale)

    while True:
      environment = rospy.get_param("/environment", None)
      if environment is not None:
        break
      rospy.sleep(1)

    self._create_world(pickle.loads(environment))

    rospy.sleep(10)
    rospy.set_param("/environment_ready", True)

    rospy.loginfo("World created")

  def _create_world(self, environment):
    self._deploy_objects(environment)
    self._deploy_obstacles(environment)
    self._deploy_robots(environment)

  def _deploy_objects(self, environment):
    for item in environment["object_list"]:
      self._deploy_item(
        thing = item["start"],
        model_name = "item_%i" % item["idx"],
        model_type = "item",
        tracking = True
      )

      self._deploy_item(
        thing = item["end"],
        model_name = "item_target_%i" % item["idx"],
        model_type = "item_target"
      )

  def _deploy_obstacles(self, environment):
    obstacle_counter = 0

    for obstacle in environment["workspace"].get_obstacles():

      obstacle_counter += 1

      self._deploy_item(
        thing = obstacle,
        model_name = "obstacle_%i" % obstacle_counter,
        model_type = "obstacle"
      )

  def _deploy_robots(self, environment):
    for robot in environment["robot_dict"].values():

      args = launcher_lib.create_args(
        x = robot.x * self.scale,
        y = robot.y * self.scale,
        z = robot.z * self.scale,
        id = robot.idx
      )

      if robot.robot_type == Robot.TYPES.land:
        launcher_lib.launch("simulation_manager", "deploy_robot.launch", args)

      elif robot.robot_type == Robot.TYPES.aerial:
        launcher_lib.launch("simulation_manager", "deploy_robot_aerial.launch", args)

  def _deploy_item(self, thing, model_name, model_type, tracking = False):

    args = launcher_lib.create_args(
      x = thing.x * self.scale,
      y = thing.y * self.scale,
      z = thing.z * self.scale,
      model_name = model_name,
      model_type = model_type
    )

    launcher_lib.launch("simulation_manager", "deploy_object.launch", args)

    if tracking:
      args = launcher_lib.create_args(
        model_name = model_name
      )

      launcher_lib.launch("tracking_system", "simulation/tracker_simulation_item.launch", args)

# ############################################################################
# Init Node

def run():
  rospy.init_node("world_controller")

  WorldManager().run()

  rospy.spin()

if __name__ == "__main__":
  try:
    run()
  except rospy.ROSInterruptException:
    pass
