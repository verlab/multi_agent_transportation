#!/usr/bin/python

import rospy

from nautilus_planner.planning.planner import Planner
from nautilus_planner.planning.planner_a_star import PlannerAStar
import nautilus_planner.utilities.heuristic as heuristic

from nautilus_planner.analyzer.analyzer import Analyzer
from nautilus_planner.allocator.allocator import Allocator

from system_utils.encode_decode_lib import decode_data
from system_utils.encode_decode_lib import encode_data

import time

def run():
  rospy.init_node("planning_manager")

  workspace = decode_data( rospy.get_param("/workspace") )

  planner = PlannerAStar()
  planner.type = Planner.types.both
  planner.heuristic = heuristic.manhattan

  start = time.clock()

  sub_time = time.clock()
  print "Planning..."
  plan = planner.plan( workspace )
  print "Planning time:", time.clock() - sub_time

  sub_time = time.clock()
  print "Analyzing..."
  segments = Analyzer.split( plan )
  checkpoints = Analyzer.create_checkpoints( segments )
  print "Analyzing time:", time.clock() - sub_time

  allocator = Allocator()

  sub_time = time.clock()
  print "Allocating..."
  move_groups, allocation_list = allocator.allocate( workspace, checkpoints )
  print "Allocating time:", time.clock() - sub_time

  print "Total time:", time.clock() - start

  allocation_data = {}

  for idx, allocation in enumerate( allocation_list ):
    parent_node, move_node = allocation
    allocation_data[idx] = [ parent_node, move_node ]

  encoded_allocation = encode_data( allocation_data )

  rospy.set_param("/allocation", encoded_allocation)
  rospy.set_param("/allocation_done", True)

  rospy.sleep(5)

  rospy.set_param("/allocation_done", False)

if __name__ == "__main__":
  try:
    run()
  except rospy.ROSInterruptException:
    pass
