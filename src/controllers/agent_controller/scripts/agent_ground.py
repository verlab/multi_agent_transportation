#!/usr/bin/python

import sys
sys.path.append("/home/ramonmelo/Dropbox/Ramon_Projects/ramon-mestrado/")

import rospy

from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Bool
from std_srvs.srv import Empty

from nautilus_lib.models.robot import Robot

import networkx as nx
import pickle

class AgentGroundController(object):
  """
  Control a Ground Agent
  """

  inf = 10000000000000

  def __init__(self, name = "robot", id = 0):
    self.id = id
    self.name = name

    # Commands
    self._goal_publisher = None
    self._reach = False

    self._pre_execute = False

  def run(self):
    rospy.loginfo("start controller for %s" % self.name)

    # Goal Publisher
    self._goal_publisher = rospy.Publisher("goal", Pose, queue_size = 1)

    # Reach Subscriber
    rospy.Subscriber("reach", Bool, self._reach_callback)

    # Service Proxy
    rospy.wait_for_service('command_to_back')
    self._back_command = rospy.ServiceProxy( "command_to_back", Empty )

    self._execute_controller()

  # Control Methods

  def _reach_callback(self, data):
    self._reach = data.data

  def _execute_controller(self):
    rate = rospy.Rate(5)

    # System scale
    self._scale = rospy.get_param("/world_scale", 1)

    # Wait until the allocation is done
    while not rospy.get_param("/allocation_done", False) and not rospy.is_shutdown():
      rospy.loginfo("wait allocation")
      rospy.sleep(1)

    # Get the allocation
    allocation_list = rospy.get_param("/allocation")
    allocation_list = pickle.loads(allocation_list)

    allocation_phases = range(len(allocation_list))

    while not rospy.is_shutdown():

      # Get current phase
      phase = rospy.get_param("/current_phase")

      if phase == -1: break

      allocation_data = allocation_list[phase]

      allocation = allocation_data['allocation']
      robot_plan = allocation_data['robot_plan']
      cost_matrix = allocation_data['cost_matrix']
      work_list = allocation_data['work_list']

      executed = False

      for robot_id, obj_id in allocation:
        if robot_id == self.id:
          cost = cost_matrix[self.id][obj_id]

          if cost != AgentGroundController.inf:
            obj = work_list[ obj_id ]

            prepare_plan = robot_plan[self.id][ obj['idx'] ]['prepare_plan']
            rospy.loginfo("execute prepare plan")

            self._execute_plan(prepare_plan)

            moviment_plan = robot_plan[self.id][ obj['idx'] ]['moviment_plan']
            rospy.loginfo("execute move plan")

            self._execute_plan(moviment_plan)

            # execute plans

            next_phase = phase + 1
            if next_phase in allocation_phases:
              phase = next_phase
            else:
              phase = -1

            rospy.loginfo("next phase: %i" % phase)
            rospy.set_param('/current_phase', phase)

            executed = True

      rospy.loginfo("wait next phase")
      while rospy.get_param("/current_phase") == phase and not executed:
        rate.sleep()

        # # Get the moviments
        # pre_move, move = allocation[str(phase)]
        # phase_robot = pre_move.robot.id

        # # If the robot is not responsible for the
        # # moviment, wait until it changes
        # if phase_robot != self.id:

        #   rospy.loginfo("I will wait, okey... ID: %s", self.id)

        #   next_phase = phase + 1

        #   while True:
        #     if str(next_phase) in allocation:

        #       next_pre_move, next_move = allocation[str(next_phase)]
        #       next_phase_robot = next_pre_move.robot.id

        #       if next_phase_robot == self.id and not self._pre_execute:
        #         self._pre_execute = True

        #         rospy.loginfo("But I will make my part")
        #         rospy.loginfo("Executing pre move")
        #         self._execute_plan( next_pre_move.plan )
        #         rospy.loginfo("Done pre move")

        #         break

        #       next_phase = next_phase + 1
        #     else:
        #       break

        #   while rospy.get_param("/current_phase") == phase:
        #     rate.sleep()

        # else:
        #   # The robot is responsible by the current phase

        #   rospy.loginfo("I will do this! ID: %s", self.id)

        #   if not self._pre_execute:
        #     rospy.loginfo("Executing pre move")
        #     self._execute_plan( pre_move.plan )
        #     rospy.loginfo("Done pre move")

        #   rospy.loginfo("Executing move")
        #   self._execute_plan( move.plan )
        #   rospy.loginfo("Done move")

        #   rospy.sleep(4)
        #   self._back_command()

        #   self._pre_execute = False

        #   phase += 1
        #   if str(phase) not in allocation:
        #     phase = -1

        #   rospy.set_param("/current_phase", phase)

      # rospy.set_param("/done", True)

    rospy.loginfo('all done by robot %s' % self.id)

  def _execute_plan(self, plan):
    rospy.loginfo("executing a plan of size: %i" % len(plan))

    path = nx.shortest_path( plan, plan.graph['start_id'], plan.graph['end_id'] )

    for node_id in path:
      node = plan.node[node_id]
      target = node['state'].position * self._scale

      goal_pose = Pose()
      goal_pose.position = Point( *target )

      rospy.loginfo(goal_pose)

      self._goal_publisher.publish( goal_pose )
      rospy.sleep(2)

      while not self._reach:
        rospy.sleep(1)

  # def _execute_plan(self, plan):
  #   timeout = 6

  #   rospy.loginfo( len(plan) )

  #   for moviment in plan:
  #     # For simulation
  #     goal = moviment.position * self._scale

  #     goal_pose = Pose()
  #     goal_pose.position = Point( *goal )

  #     self._goal_publisher.publish( goal_pose )

  #     rospy.sleep(2)
  #     time = timeout
  #     while not self._reach and time > 0:
  #       rospy.sleep(1)
  #       time -= 1

def run():
  rospy.init_node("agent_ground_controller", anonymous = True)

  name = rospy.get_param("~robot_prefix")
  id = rospy.get_param("~robot_id")

  AgentGroundController( name = name, id = id ).run()

  rospy.spin()

if __name__ == "__main__":
  try:
    run()
  except rospy.ROSInterruptException, e:
    pass
