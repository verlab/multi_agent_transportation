#!/usr/bin/python

import numpy as np
import rospy
import simplejson as json

from geometry_msgs.msg import Pose
from std_msgs.msg import String

from nautilus_planner.environment.map import Map
from nautilus_planner.environment.thing import Thing

import pygame
from pygame.locals import *

display = None
workspace = None

class RealWorkscapeLib(object):

  def __init__(self):
    self.item_topic = "/item/current_pose"
    self.item_target_topic = "/item_target/current_pose"
    self.obstacle_topic = "/obstacle_info"

  def create_environment(self):

    # Item Initial and Final Positions

    x_start = self.item_position.x
    y_start = self.item_position.y

    x_end = self.item_target_position.x
    y_end = self.item_target_position.y

    initial_state = Thing(
      position = ( x_start, y_start, 0.5 ),
      size = ( 1, 1, 1 ),
      rotation = 0
    )

    goal_state = initial_state.copy()
    goal_state.position = ( x_end, y_end, 0.5 )

    width = 1
    height = 1
    depth = 1

    # Maps
    maps = Map( size = (20, 20, 50) )

    # Obstacles

    cfg = {
      "33": {
        'w': 6,
        'h': 0
      },
      "27": {
        'w': 0,
        'h': 4.1
      },
      "49": {
        'w': 9,
        'h': 0
      },
      "79": {
        'w': 9,
        'h': 0
      },
      "48": {
        'w': 0,
        'h': 2.5
      },
      "54": {
        'w': 4,
        'h': 0
      }
    }

    for obstacle_id in self.obstacle_position:

      data = self.obstacle_position[ obstacle_id ]

      x = data["x"]
      y = data["y"]
      z = 0.5

      width = 1
      height = 1

      if obstacle_id in cfg:
        config = cfg[obstacle_id]

        width += config["w"]
        height += config["h"]

      obstacle = Thing(
        position = ( x, y, z ),
        size     = ( width, height, depth )
      )

      maps.add_obstacle( obstacle )

    return {
      "initial_state": initial_state,
      "goal_state": goal_state,
      "maps": maps
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

plus_scale = np.array([1,1,0])
mult_scale = np.array([100,100,1])
adjust_scale = np.array([20.5,0,0])

def up_scale( position ):
  new_pos = position.copy()

  new_pos += plus_scale
  new_pos *= mult_scale
  new_pos -= adjust_scale

  return new_pos

def draw_obstacles():
  global workspace
  global display

  environment = workspace.create_environment()

  for obstacle in environment["maps"].obstacles:
    x, y, z = up_scale( obstacle.position )

    w, h, d = obstacle.size * 10

    quad = pygame.Rect(x, y, w, h)
    pygame.draw.rect( display, (0,0,0), quad )

def run():
  global display
  global workspace

  rospy.init_node("environment_viewer")

  workspace = RealWorkscapeLib()
  workspace.run()

  rospy.sleep(5)

  pygame.init()

  display = pygame.display.set_mode( (800, 600) )

  clock = pygame.time.Clock()
  pygame.display.set_caption("Environment Viewer")
  running = True

  while running:
    # Events
    events = pygame.event.get()

    for event in events:
      if event.type == pygame.QUIT:
        running = False

      # process_event( event )

    # Update Draw
    display.fill( (255, 255, 255) )

    draw_obstacles()

    pygame.display.update()
    clock.tick( 1 )

if __name__ == "__main__":
  try:
    run()
  except rospy.ROSInterruptException:
    pass
