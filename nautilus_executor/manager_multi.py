#!/usr/bin/python

from manager import Manager
from nautilus_lib.planner import Planner
from nautilus_lib.models.robot import Robot

import numpy as np
from munkres import Munkres

class ManagerHungarianMulti(Manager):

    inf = 10000000000000

    @staticmethod
    def execute( environment ):
        # print "-------------------------------------------------"
        # print "Planning for objects..."

        Manager._create_object_plan( environment )

        allocation_list = []

        space = environment['workspace']
        object_list = environment['object_list']
        robots = environment['robot_dict']

        total_object = 0
        total_robot = 0

        allocator = Munkres()

        # While not all objects are transported
        while True:
            # Create the list with objects that there are not yet
            # fully transported
            work_list = [ obj for obj in object_list if len(obj['plan']['segments']) != 0 ]

            # With all objects were transported, finish
            if len(work_list) == 0:
                break

            # print "-------------------------------------------------"
            # print "Planning for robots..."

            # Create the allocation matrix
            # Square, for the Hungarian Method
            shape = (max( len( work_list ), len( robots ) ), ) * 2
            cost_matrix = np.full( shape, ManagerHungarianMulti.inf, dtype=np.int )

            robot_id_list = robots.keys()
            robot_plan = {}

            # For each robot available
            for robot_index, robot_id in enumerate( robot_id_list ):

                # print "# robot", robot_index
                robot = robots[ robot_id ]
                robot_plan[ robot_id ] = {}

                # For each object to be transported
                for obj_index, obj in enumerate( work_list ):
                    # print "### object", obj['idx']

                    robot_plan[ robot_id ][ obj['idx'] ] = {}

                    # Get first segment
                    segment = obj['plan']['segments'][0]

                    if segment.graph['type'] is not robot.robot_type:
                        continue

                    # If there is another object in the segment,
                    # its no possible to be executed
                    in_place_objects = [ item for item in object_list if (item['idx'] is not obj['idx']) and (hash( item['current_pos'] ) in segment) ]

                    if len( in_place_objects ) != 0:
                        continue

                    segment_start = segment.node[ segment.graph['start_id'] ]
                    segment_end = segment.node[ segment.graph['end_id'] ]

                    # Prepare Movement

                    space.temp_obstacle_list = [ item['current_pos'] for item in object_list ]

                    robot_start = robot.copy()

                    robot_end = robot.copy()

                    if robot.robot_type == Robot.TYPES.aerial:
                        robot_end.position = Planner.create_robot_position( segment_start, -1 ).position
                    else:
                        robot_end.position = Planner.create_robot_position( segment_start, 2 ).position

                    try:
                        prepare_plan = Manager._execute_planning({
                            'start_position': robot_start,
                            'end_position': robot_end,
                            'workspace': space,
                            'robot_type': robot.robot_type
                        })
                    except Exception, e:
                        print "Object: ", obj['idx'] + 1
                        print "Impossible prepare plan"
                        continue

                    space.temp_obstacle_list = []

                    robot_plan[ robot_id ][ obj['idx'] ][ 'prepare_plan' ] = prepare_plan

                    # Move Movement

                    space.temp_obstacle_list = [ item['current_pos'] for item in object_list if item['idx'] is not obj['idx'] ]

                    robot_start.position = robot_end.position
                    robot_end = robot.copy()
                    robot_end.position = Planner.create_robot_position( segment_end ).position

                    try:
                        moviment_plan = Manager._execute_planning({
                            'start_position': robot_start,
                            'end_position': robot_end,
                            'workspace': space,
                            'robot_type': robot.robot_type
                        })
                    except Exception, e:
                        print "Object: ", obj['idx'] + 1
                        print "Impossible movement plan"
                        continue

                    space.temp_obstacle_list = []

                    robot_plan[ robot_id ][ obj['idx'] ][ 'moviment_plan' ] = moviment_plan

                    # Total cost to movement this object
                    # by the current robot
                    total_cost = obj['plan']['total'] - len(segment) + len(prepare_plan)

                    # Populate the cost matrix
                    cost_matrix[ robot_index ][ obj_index ] = total_cost

            # print "-------------------------------------------------"
            # print "Allocating..."

            # Create the allocation
            allocation = allocator.compute( cost_matrix.tolist() )

            for robot_index, obj_index in allocation:
                value = cost_matrix[robot_index][obj_index]

                # If the robot was allocated to one object
                if value != ManagerHungarianMulti.inf:
                    # print "-----------"
                    # print "# robot", robot_index
                    # print "## object", obj_index

                    robot_id = robot_id_list[ robot_index ]
                    robot = robots[ robot_id ]
                    obj = work_list[ obj_index ]

                    segment = obj['plan']['segments'].pop(0)
                    segment_end = segment.node[ segment.graph['end_id'] ]

                    robot.position = Planner.create_robot_position( segment_end ).position

                    obj['current_pos'].position = segment_end['state'].position
                    obj['plan']['total'] -= len(segment)

                    prepare_plan = robot_plan[ robot_id ][ obj['idx'] ]['prepare_plan']
                    moviment_plan = robot_plan[ robot_id ][ obj['idx'] ]['moviment_plan']

                    move_size = len(prepare_plan) + len(moviment_plan)

                    Manager.robot_path.append( prepare_plan )
                    Manager.robot_path.append( moviment_plan )

                    total_object += len( segment )
                    total_robot += move_size

            allocation_list.append({
                'allocation': allocation,
                'robot_plan': robot_plan,
                'cost_matrix': cost_matrix,
                'work_list': work_list
            })

        print "Total object:", total_object
        print "Total robot:", total_robot
        print "Total:", (total_object + total_robot)

        # return total_object, total_robot
        return allocation_list
