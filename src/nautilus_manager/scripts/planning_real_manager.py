#!/usr/bin/python

import rospy

from nautilus_planner.planning.planner import Planner
from nautilus_planner.planning.planner_a_star import PlannerAStar
import nautilus_planner.utilities.heuristic as heuristic

from nautilus_planner.analyzer.analyzer import Analyzer
from nautilus_planner.allocator.allocator import Allocator

from system_utils.encode_decode_lib import decode_data
from system_utils.encode_decode_lib import encode_data

from system_utils.math_lib import up_scale, down_scale

from nautilus_planner.view_data import viewer

import time

import numpy as np

def info( *msg ):
  # print msg
  rospy.loginfo( msg )

def run():
  rospy.init_node("planning_manager")

  # Test
  environment_saver = '{"goal_state":{"__Thing__":{"solid":true,"position":[-0.20987670123577118,-0.14046016335487366,0.5],"rotation":0,"size":[0.9,0.9,1.0]}},"robot_descriptors":[{"__Robot__": {"robot_type": 0, "solid": true, "position": [0.7113677859306335,0.6788556575775146, 0.5], "rotation": 0, "id": 0, "size": [1.0, 1.0, 1.0]}}, {"__Robot__":{"robot_type": 0, "solid": true, "position": [0.1615365892648697, 0.6750851273536682,0.5], "rotation": 0, "id": 3, "size": [1.0, 1.0, 1.0]}}, {"__Robot__": {"robot_type":0, "solid": true, "position": [-0.23987910151481628, 0.6611530780792236, 0.5], "rotation":0, "id": 4, "size": [1.0, 1.0, 1.0]}}],"initial_state":{"__Thing__":{"solid":true,"position":[-0.26382121443748474,0.3627174496650696,0.5],"rotation":0,"size":[0.9,0.9,1.0]}},"maps":{"__Map__":{"obstacles":[{"solid":true,"position":[0.4542118310928345,0.27095896005630493,0.5],"rotation":0.0,"size":[1.0,6.3648,1.0]},{"solid":true,"position":[0.23283937573432922,0.12851622700691223,0.5],"rotation":0.0,"size":[1.0,3.5,1.0]},{"solid":true,"position":[-0.27195894718170166,0.25328877568244934,0.5],"rotation":0.0,"size":[10.0,1.0,1.0]},{"solid":true,"position":[0.12414830923080444,-0.2529858946800232,0.5],"rotation":0.0,"size":[7.0,1.0,1.0]},{"solid":true,"position":[-0.012492849491536617,0.4815795123577118,0.5],"rotation":0.0,"size":[5.0,1.0,1.0]},{"solid":true,"position":[-0.24814337491989136,-0.010411872528493404,0.5],"rotation":0.0,"size":[10.0,1.0,1.0]}],"size":[20.0,20.0,50.0]}}}'

  # Valid
  # environment_saver = '{"goal_state":{"__Thing__":{"solid":true,"position":[-0.1762319654226303,-0.11201760172843933,0.5],"rotation":0,"size":[1.0,1.0,1.0]}},"robot_descriptors":[{"__Robot__":{"robot_type":0,"solid":true,"position":[0.07933227717876434,0.7094988822937012,0.5],"rotation":0,"id":3,"size":[1.0,1.0,1.0]}},{"__Robot__":{"robot_type":0,"solid":true,"position":[-0.41981786489486694,0.7608802318572998,0.5],"rotation":0,"id":4,"size":[1.0,1.0,1.0]}}],"initial_state":{"__Thing__":{"solid":true,"position":[-0.21476462483406067,0.38498061895370483,0.5],"rotation":0,"size":[1.0,1.0,1.0]}},"maps":{"__Map__":{"obstacles":[{"solid":true,"position":[-0.497730553150177,-0.004038929007947445,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[-0.26175811886787415,0.2699475884437561,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[0.45928677916526794,0.4145994186401367,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[-0.17596259713172913,0.27162206172943115,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[0.23968994617462158,0.19864653050899506,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[-0.09241080284118652,0.2746395170688629,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[0.4515843987464905,0.5011248588562012,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[0.23570390045642853,0.2837114632129669,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[-0.021415265277028084,-0.23099710047245026,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[-0.008958060294389725,0.2740612328052521,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[0.06647872179746628,-0.22988274693489075,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[0.23500441014766693,0.01793639548122883,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[0.045459847897291183,0.51691734790802,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[-0.10755123198032379,-0.23776188492774963,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[0.12845483422279358,0.513974666595459,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[0.3416345417499542,-0.22144801914691925,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[0.24821166694164276,-0.2239764928817749,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[0.08372170478105545,0.013063889928162098,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[-0.6590416431427002,-0.009435389190912247,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[1.76470148289809e-05,0.009700312279164791,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[-0.16590209305286407,0.0023806090466678143,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[-0.0830417051911354,0.005631834268569946,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[0.0730472058057785,0.27350133657455444,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[0.15416377782821655,-0.2227102369070053,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[0.20939922332763672,0.5101251006126404,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[-0.1837066113948822,-0.23193489015102386,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[-0.25223734974861145,0.0015217203181236982,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[-0.21145831048488617,0.49334031343460083,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[-0.5616495013237,-0.007230411749333143,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[-0.1307552307844162,0.5119829773902893,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[-0.5951478481292725,0.262834757566452,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[0.4711741507053375,0.14485639333724976,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[-0.6500034332275391,0.25203195214271545,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[0.15223830938339233,0.2755989134311676,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[0.16491493582725525,0.015520848333835602,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[0.24270926415920258,0.10602869838476181,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[0.4770439863204956,0.0596420019865036,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[-0.34775230288505554,0.2699204385280609,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[-0.49453306198120117,0.25175371766090393,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[-0.3222827613353729,-0.0002889450697693974,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[0.46598073840141296,0.3278120160102844,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[-0.43419772386550903,0.26752108335494995,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[-0.4011700451374054,-0.002311522839590907,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[0.4822442829608917,0.2426251471042633,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]},{"solid":true,"position":[-0.042947493493556976,0.505846381187439,0.5],"rotation":0.0,"size":[1.0,1.0,1.0]}],"size":[50.0,50.0,50.0]}}}'

  environment_saver = decode_data( environment_saver )

  environment = decode_data( rospy.get_param("/environment") )

  environment["goal_state"] = environment_saver["goal_state"]
  environment["initial_state"] = environment_saver["initial_state"]

  environment["goal_state"].position = up_scale( environment["goal_state"].position )
  environment["initial_state"].position = up_scale( environment["initial_state"].position )

  # for robot in environment["robot_descriptors"]:
    # robot.position = up_scale( robot.position )

  for robot in environment_saver["robot_descriptors"]:
    robot.position = up_scale( robot.position )

  environment["robot_descriptors"] = environment_saver["robot_descriptors"]

  for obstacle in environment_saver["maps"].obstacles:
    obstacle.position = up_scale( obstacle.position )

    environment["maps"].add_obstacle( obstacle )

  # for obstacle in environment["maps"].obstacles:
    # obstacle.position = up_scale( obstacle.position )

  planner = PlannerAStar( move_size = 0.1 )
  planner.type = Planner.types.land
  planner.heuristic = heuristic.manhattan

  start = time.clock()

  # Planner
  sub_time = time.clock()
  info("Planning...")

  plan = planner.plan( environment )
  info( len(plan), "plan step(s)" )

  info( "Planning time:", time.clock() - sub_time )

  # Analyzer
  sub_time = time.clock()
  info( "Analyzing..." )
  segments = Analyzer.split( plan )

  info( len(segments), "segment(s)" )

  checkpoints = Analyzer.create_checkpoints( segments )

  info( len(checkpoints), "checkpoint(s)" )

  info( "Analyzing time:", time.clock() - sub_time )

  viewer.view_plan( environment["maps"], plan, checkpoints )

  # Allocator
  allocator = Allocator( move_size = 0.5 )

  sub_time = time.clock()
  info( "Allocating..." )

  move_groups, allocation_list = allocator.allocate( environment, checkpoints )
  info( "Allocating time:", time.clock() - sub_time )

  info( "Total time:", time.clock() - start )

  allocation_data = {}

  for idx, allocation in enumerate( allocation_list ):
    parent_node, move_node = allocation
    allocation_data[idx] = [ parent_node, move_node ]

  encoded_allocation = encode_data( allocation_data )

  # print encoded_allocation

  rospy.set_param("/allocation", encoded_allocation)

  viewer.view_plan( environment["maps"], plan, checkpoints, allocation_data )

  rospy.set_param("/allocation_done", True)

  info( "Allocation done!" )

if __name__ == "__main__":
  try:
    run()
  except rospy.ROSInterruptException:
    pass
