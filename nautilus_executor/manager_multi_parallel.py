#!/usr/bin/python

from manager import Manager
from nautilus_lib.planner import Planner
from nautilus_lib.analyzer import Analyser

import numpy as np
from munkres import Munkres

from multiprocessing import Pool

def _object_plan( args ):
    obj, object_list, space, robot_types = args

    space.temp_obstacle_list = [ item['start'] for item in object_list if item['idx'] is not obj['idx'] ]

    test_case = {
        'start_position': obj['start'],
        'end_position': obj['end'],
        'workspace': space,
        'robot_types': robot_types
    }

    try:
        path = Manager._execute_planning( test_case )
        segments = Analyser.slipt( path )

        obj['plan'] = {
            'path': path,
            'segments': segments,
            'total': len(path)
        }

    except Exception, e:
        print "Object: ", obj['idx'] + 1
        print e.message

    space.temp_obstacle_list = []

    return obj

def _robot_plan( args ):
    robot_index, robot, cost_row, work_list, object_list, space = args

    robot_plan = {}

    for obj_index, obj in enumerate( work_list ):
        robot_plan[ obj['idx'] ] = {}

        segment = obj['plan']['segments'][0]

        in_place_objects = [ item for item in object_list if (item['idx'] is not obj['idx']) and (hash( item['current_pos'] ) in segment) ]

        if len( in_place_objects ) != 0:
            print 'obj', obj['idx'] + 1, 'impossible by', [ item['idx'] + 1 for item in in_place_objects ]
            continue

        segment_start = segment.node[ segment.graph['start_id'] ]
        segment_end = segment.node[ segment.graph['end_id'] ]

        # Prepare Movement

        space.temp_obstacle_list = [ item['current_pos'] for item in object_list ]

        robot_start = robot.copy()

        robot_end = robot.copy()
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

        robot_plan[ obj['idx'] ][ 'prepare_plan' ] = prepare_plan

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

        robot_plan[ obj['idx'] ][ 'moviment_plan' ] = moviment_plan

        total_cost = obj['plan']['total'] - len(segment) + len(prepare_plan)

        cost_row[ obj_index ] = total_cost

    return robot_index, robot.idx, robot_plan, cost_row

class ManagerParallel(Manager):

    inf = 100000000000000

    @staticmethod
    def _parallel_object_plan( environment ):
        object_list = environment['object_list']
        space = environment['workspace']
        robot_types = list(set([ robot.robot_type for robot in environment['robot_dict'].values() ]))

        pool = Pool()
        arg_list = []

        for obj in object_list:
            arg_list.append( (obj, object_list, space.copy(), robot_types) )

        result = pool.map( _object_plan, arg_list )

        pool.close()
        pool.join()

        return result

    @staticmethod
    def execute( environment ):
        object_list = ManagerParallel._parallel_object_plan( environment )

        space = environment['workspace']
        robots = environment['robot_dict']

        total_object = 0
        total_robot = 0

        allocator = Munkres()

        planner_poll = Pool()

        while True:

            work_list = [ obj for obj in object_list if len(obj['plan']['segments']) != 0 ]

            if len(work_list) == 0:
                break

            size = max( len( work_list ), len( robots ) )
            shape = (size, ) * 2
            cost_matrix = np.full( shape, ManagerParallel.inf, dtype=np.int )

            robot_id_list = robots.keys()
            robot_global_plan = {}

            arg_list = []

            for robot_index, robot_id in enumerate( robot_id_list ):
                robot = robots[ robot_id ]

                cost_row = np.full( (size,), ManagerParallel.inf, dtype=np.int )

                arg_list.append( ( robot_index, robot, cost_row, work_list, object_list, space.copy() ) )

            result = planner_poll.map( _robot_plan, arg_list )

            for data in result:
                robot_index, robot_id, robot_plan, cost_row = data

                cost_matrix[ robot_index ] = cost_row
                robot_global_plan[ robot_id ] = robot_plan

            allocation = allocator.compute( cost_matrix.tolist() )

            for robot_index, obj_index in allocation:
                value = cost_matrix[robot_index][obj_index]

                # If the robot was allocated to one object
                if value != ManagerParallel.inf:
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

                    prepare_plan = robot_global_plan[ robot_id ][ obj['idx'] ]['prepare_plan']
                    moviment_plan = robot_global_plan[ robot_id ][ obj['idx'] ]['moviment_plan']

                    move_size = len(prepare_plan) + len(moviment_plan)

                    # Manager.robot_path.append( prepare_plan )
                    # Manager.robot_path.append( moviment_plan )

                    total_object += len( segment )
                    total_robot += move_size

        planner_poll.close()
        planner_poll.join()

        # print "Total object:", total_object
        # print "Total robot:", total_robot
        # print "Total:", (total_object + total_robot)

        return total_object, total_robot
