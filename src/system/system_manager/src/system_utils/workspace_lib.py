
import rospy

from geometry_msgs.msg import Pose
from std_msgs.msg import String

import simplejson as json
import cPickle as pickle

from nautilus_planner.environment.map import Map
from nautilus_planner.environment.thing import Thing
from nautilus_planner.environment.robot import Robot

def create_environment_configuration( config_file ):
  """
  Create a plan configuration based on a file

  Params:

    config_file : Path to configuration file

  Return:
    A dictionary with all configurations
  """

  config_dict = json.load( open( config_file ) )

  # Create the Map
  maps = Map( config_file = config_file )

  # Create the Item Descriptions

  object_descriptors = []

  if "object_list" in config_dict:
    for item in config_dict["object_list"]:

      idx = int( item["id"] )

      width = float( item["width"] )
      height = float( item["height"] )
      depth = float( item["depth"] )

      start_x = float( item["start"]["x"] )
      start_y = float( item["start"]["y"] )

      end_x = float( item["end"]["x"] )
      end_y = float( item["end"]["y"] )

      z = depth / 2

      initial_state = Thing(
        position = ( start_x, start_y, z ),
        size = ( width, height, depth )
      )

      goal_state = Thing(
        position = ( end_x, end_y, z ),
        size = ( width, height, depth )
      )

      object_description = {
        "id": idx,
        "start": initial_state,
        "end": goal_state
      }

      object_descriptors.append( object_description )

  # Create Robot Configurations

  robot_descriptors = []

  if "robot_list" in config_dict:
    for i, robot in enumerate( config_dict["robot_list"] ):

      width = float( robot["width"] )
      height = float( robot["height"] )
      depth = float( robot["depth"] )

      x = float( robot["x"] )
      y = float( robot["y"] )
      z = depth / 2

      robot_descriptor = Robot(
        position = ( x, y, z ),
        size = ( width, height, depth ),
        robot_type = int( robot["type"] ),
        id = i
      )

      robot_descriptors.append( robot_descriptor )

  return {
    "maps": maps,
    "robot_descriptors": robot_descriptors,
    "object_descriptors": object_descriptors
  }

# REAL ENVIRONMENT

import simplejson as json

class RobotTopicSubscriber(object):

  def __init__(self, robot_id, robot_topic):
    self.robot_id = robot_id
    self.robot_topic = robot_topic

  def get_data(self):
    return {
      "robot_id": self.robot_id,
      "robot_position": self.robot_position
    }

  def robot_callback(self, data):
    self.robot_position = data.position

  def run(self):
    rospy.Subscriber( self.robot_topic, Pose, self.robot_callback )

    return self

class RealWorkscapeLib(object):

  def __init__(self, robot_id_list):
    self.item_topic = "/item/current_pose"
    self.item_target_topic = "/item_target/current_pose"

    self.robot_data = {}
    self.robot_listeners = []

    for robot_id in robot_id_list:
      self.robot_data[ robot_id ] = "/robot_%s/mobile_base/current_pose" % ( robot_id )

    self.obstacle_topic = "/obstacle_info"

  def create_environment_configuration(self):

    # Item Initial and Final Positions

    # x_start = self.item_position.x
    # y_start = self.item_position.y

    # x_end = self.item_target_position.x
    # y_end = self.item_target_position.y

    # initial_state = Thing(
    #   position = ( x_start, y_start, 0.5 ),
    #   size = ( 1, 1, 1 ),
    #   rotation = 0
    # )

    # goal_state = initial_state.copy()
    # goal_state.position = ( x_end, y_end, 0.5 )

    width = 1
    height = 1
    depth = 1

    # Robots

    robot_descriptors = []

    for listener in self.robot_listeners:
      data = listener.get_data()

      x = data["robot_position"].x
      y = data["robot_position"].y
      z = 0.5

      robot_descriptor = Robot(
        position = ( x, y, z ),
        size = ( width, height, depth ),
        rotation = 0,
        robot_type = 0,
        id = data["robot_id"]
      )

      robot_descriptors.append( robot_descriptor )

    # Maps
    maps = Map( size = (20, 20, 50) )

    # Obstacles

    # cfg = {
    #   "33": {
    #     'w': 6,
    #     'h': 0
    #   },
    #   "27": {
    #     'w': 0,
    #     'h': 4.1
    #   },
    #   "49": {
    #     'w': 9,
    #     'h': 0
    #   },
    #   "79": {
    #     'w': 9,
    #     'h': 0
    #   },
    #   "48": {
    #     'w': 0,
    #     'h': 2.5
    #   },
    #   "54": {
    #     'w': 4,
    #     'h': 0
    #   }
    # }

    # for obstacle_id in self.obstacle_position:

    #   data = self.obstacle_position[ obstacle_id ]

    #   x = data["x"]
    #   y = data["y"]
    #   z = 0.5

    #   width = 1
    #   height = 1

    #   if obstacle_id in cfg:
    #     config = cfg[obstacle_id]

    #     width += config["w"]
    #     height += config["h"]

    #   obstacle = Thing(
    #     position = ( x, y, z ),
    #     size     = ( width, height, depth )
    #   )

    #   maps.add_obstacle( obstacle )

    return {
      # "initial_state": initial_state,
      # "goal_state": goal_state,
      "maps": maps,
      "robot_descriptors": robot_descriptors
    }

  def item_callback( self, data ):
    self.item_position = data.position

  def item_target_callback( self, data ):
    self.item_target_position = data.position

  def obstacle_callback( self, data ):
    self.obstacle_position = json.loads( data.data )

  def run(self):
    rospy.Subscriber( self.item_topic, Pose, self.item_callback )
    rospy.Subscriber( self.item_target_topic, Pose, self.item_target_callback )
    rospy.Subscriber( self.obstacle_topic, String, self.obstacle_callback )

    for robot_id in self.robot_data:
      listener = RobotTopicSubscriber( robot_id, self.robot_data[robot_id] ).run()
      self.robot_listeners.append( listener )
