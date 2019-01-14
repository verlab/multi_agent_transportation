#!/usr/bin/python

import rospy

from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Pose, Twist
from tf.transformations import euler_from_quaternion

import numpy as np
import sympy as sp
import sympy.mpmath as mp

from agent_utils.utils import *

class AgentGroundVelocityController(object):
  """
  Control the velocities of the mobile base
  to go from currento position to a goal position
  """

  config = {
    "real": {
      "tolerance": 0.01,
      "p": 0.4,
      "i": 0.05, # 0.1
      # "linear": 8,
      # "angular": -0.3, # -0.2
      "angular_diff": 0.15, # 0.1
      "min_linear_vel": 0.1,
      "min_angular_vel": 0.01
    },
    "simulation": {
      "tolerance": 0.02,
      "p": 0.2,
      "i": 0.3,
      # "linear": 0.8,
      # "angular": 1,
      "angular_diff": 0.01, # 0.005
      "min_linear_vel": 0.1,
      "min_angular_vel": 0.02
    }
  }

  current_config = config["simulation"]

  def __init__(self):
    # Control
    self._current_pose = None
    self._command_velocity_publisher = None
    self._goal = None
    self._tolerance = self.current_config["tolerance"]
    self._reach = False

    # Used on controller
    self._old_linear_vel = 0
    self._old_angular_vel = 0

    # Run Controller
    self._enabled = False

  def run(self):
    """
    Run the controller
    """

    rospy.Subscriber("current_pose", Pose, self._current_pose_callback)
    rospy.Subscriber("goal", Pose, self._goal_callback)

    rospy.Service("command_to_back", Empty, self.command_to_back_service)

    # Controller publisher
    self._command_velocity_publisher = rospy.Publisher( "cmd_vel", Twist, queue_size = 10 )
    self._reach_publisher = rospy.Publisher("reach", Bool, queue_size = 1)

    # Execute the controller
    # waiting for goal positions
    self._execute_controller()

  # Properties

  @property
  def reach(self):
    return self._reach

  @reach.setter
  def reach(self, value):
    self._reach = value

  # Callbacks

  def _goal_callback(self, data):
    self._goal = simplify_pose( data )

    self._enabled = True
    self._reach = False

  def _current_pose_callback(self, data):
    self._current_pose = simplify_pose( data )

  # Services

  def command_to_back_service(self, req):
    self._command_to_back()
    return EmptyResponse()

  # Control Methods

  def _execute_controller(self):

    rate = rospy.Rate(10)

    P, I = sp.var('p i')
    min_linear_vel = sp.var("minLinearVel")
    min_angular_vel = sp.var("minAngularVel")

    distErr, lastLinearOut = sp.var('distErr lastLinearOut')
    anglErr, lastAngularOut, anglDiff = sp.var('anglErr lastAngularOut anglDiff')

    self.linear_eq = sp.Piecewise(
      (sp.Max( P * distErr + I * lastLinearOut, min_linear_vel ), sp.Abs( anglErr ) < anglDiff),
      (0, sp.Abs( anglErr ) >= anglDiff)
    )

    angular_eq_temp = P * anglErr + I * lastAngularOut

    self.angular_eq = sp.Piecewise(
      ( sp.Min( angular_eq_temp, -min_angular_vel ), angular_eq_temp < 0 ),
      ( sp.Max( angular_eq_temp,  min_angular_vel ), angular_eq_temp >= 0)
    )

    constansts = {
      P: self.current_config["p"],
      I: self.current_config["i"],
      min_linear_vel: self.current_config["min_linear_vel"],
      min_angular_vel: self.current_config["min_angular_vel"],
      anglDiff: self.current_config["angular_diff"]
    }

    variables = [ distErr, lastLinearOut, anglErr, lastAngularOut ]

    self.linear_eq = self.linear_eq.subs( constansts )
    self.angular_eq = self.angular_eq.subs( constansts )

    while not rospy.is_shutdown():

      if self._goal != None and self._enabled:
        self._command_to( self._goal, variables )

      self._reach_publisher.publish( Bool( self._reach ) )

      rate.sleep()

  def _command_to_back(self, times = 3):
    self._enabled = False

    time_count = times # times going back

    while time_count > 0:
      self._publish_velocity( -0.1, 0 )
      time_count -= 1

      rospy.sleep(1)

    self._publish_velocity(0, 0)

  def _command_to(self, goal, variables):

    # Get origin and target points
    curr_position = [ self._current_pose["x"], self._current_pose["y"] ]
    goal_position = [ goal["x"], goal["y"] ]

    # Get origin and target orientation
    curr_theta = rad_to_deg( self._current_pose["theta"] )
    goal_theta = angle_by_point( curr_position, goal_position )

    # Diff between current and target direction
    diff_angle = deg_to_rad( cut_angle_deg( goal_theta - curr_theta ) )
    diff_distance = distance_by_point( curr_position, goal_position )

    distErr, lastLinearOut, anglErr, lastAngularOut = variables

    linear_vel = float( self.linear_eq.subs({
      distErr: diff_distance,
      lastLinearOut: self._old_linear_vel,
      anglErr: diff_angle
    }) )

    angular_vel = float( self.angular_eq.subs({
      anglErr: diff_angle,
      lastAngularOut: self._old_angular_vel
    }) )

    self._old_linear_vel = linear_vel
    self._old_angular_vel = angular_vel

    # Flag if reach the point
    self._reach = (diff_distance <= self._tolerance)

    if self._reach:
      linear_vel = 0
      angular_vel = 0

      self._enabled = False

    # Send to mobile base
    self._publish_velocity( linear_vel, angular_vel )

  def _publish_velocity(self, linear_vel, angular_vel):
    cmd_vel = Twist()
    cmd_vel.linear.x = linear_vel
    cmd_vel.angular.z = angular_vel

    self._command_velocity_publisher.publish( cmd_vel )

def run():
  rospy.init_node("agent_ground_controller_velocity", anonymous = True)
  AgentGroundVelocityController().run()

  rospy.spin()

if __name__ == "__main__":
  try:
    run()
  except rospy.ROSInterruptException, e:
    pass
