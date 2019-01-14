#!/usr/bin/python

import rospy

from std_msgs.msg import Int16
from geometry_msgs.msg import Pose, Point
from gazebo_msgs.msg import ModelState

from agent_utils.utils import *

class AgentAerialGripperController(object):

  def __init__(self):
    self._current_pose = None
    self._item_position_publisher = None
    self._grab = False
    self._item_target = -1

  def run(self):

    rospy.Subscriber("current_pose", Pose, self._current_pose_callback)
    rospy.Subscriber("grab_item", Int16, self._grab_callback)
    rospy.Subscriber("release_item", Point, self._release_callback)

    self._item_position_publisher = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size = 10)

    self._execute_controller()

  def _current_pose_callback(self, data):
    self._current_pose = simplify_pose( data )

  def _release_callback(self, data):
    self._grab = False

    cmd = ModelState()

    cmd.model_name = 'item_%i' % self._item_target
    cmd.pose.position.x = data.x
    cmd.pose.position.y = data.y
    cmd.pose.position.z = 0.2 # data.z
    cmd.reference_frame = 'world'

    self._item_position_publisher.publish(cmd)

  def _grab_callback(self, data):
    idx = int(data.data)

    if idx != -1:
      self._item_target = idx
      self._grab = True
    else:
      self._grab = False

  def _execute_controller(self):
    rate = rospy.Rate(100)

    cmd = ModelState()

    cmd.model_name = ""
    cmd.pose.position.x = 0
    cmd.pose.position.y = 0
    cmd.pose.position.z = 0
    cmd.reference_frame = 'world'

    while not rospy.is_shutdown():

      if self._grab:
        cmd.model_name = 'item_%i' % self._item_target

        cmd.pose.position.x = self._current_pose['x']
        cmd.pose.position.y = self._current_pose['y']
        cmd.pose.position.z = self._current_pose['z'] - 0.25

        self._item_position_publisher.publish(cmd)

      rate.sleep()

def run():
  rospy.init_node("agent_aerial_controller_gripper", anonymous = True)

  AgentAerialGripperController().run()

  rospy.spin()

if __name__ == "__main__":
  try:
    run()
  except rospy.ROSInterruptException, e:
    pass
