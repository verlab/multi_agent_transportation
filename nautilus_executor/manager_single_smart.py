#!/usr/bin/python

from manager import Manager
from nautilus_lib.planner import Planner

class ManagerSmart(Manager):

    @staticmethod
    def execute( environment ):
        Manager._create_object_plan( environment )

        space = environment['workspace']
        object_list = environment['object_list']
        robot = environment['robot_dict'].values()[0]

        total_object = 0
        total_robot = 0

        while True:
            work_list = [ obj for obj in object_list if len(obj['plan']['segments']) != 0 ]

            if len(work_list) == 0:
                break

            best_obj = None
            best_cost = 0
            best_move_size = 0

            best_prepare = None
            best_move = None

            for obj in work_list:
                segment = obj['plan']['segments'][0]

                # If there is another object in the segment,
                # its no possible to be executed

                in_place_objects = [ item for item in object_list if (item['idx'] is not obj['idx']) and (hash( item['current_pos'] ) in segment) ]

                if len( in_place_objects ) != 0:
                    print 'obj', obj['idx'], 'impossible by', [ item['idx'] for item in in_place_objects ]
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

                # total_cost = len(prepare_plan) - obj['plan']['total']
                total_cost = obj['plan']['total'] - len(segment) + len(prepare_plan)
                # total_cost = len(prepare_plan) + len(moviment_plan) - ( obj['plan']['total'] - len(segment) )
                # total_cost = -obj['plan']['total'] - len(segment) + len(prepare_plan)

                if best_obj is None or total_cost < best_cost:
                    best_obj = obj
                    best_cost = total_cost
                    best_move_size = len(prepare_plan) + len(moviment_plan)

                    best_prepare = prepare_plan
                    best_move = moviment_plan

            # Transport Object

            segment = best_obj['plan']['segments'].pop(0)

            # segment_start = segment.node[ segment.graph['start_id'] ]
            segment_end = segment.node[ segment.graph['end_id'] ]

            robot.position = Planner.create_robot_position( segment_end ).position
            best_obj['current_pos'].position = segment_end['state'].position
            best_obj['plan']['total'] -= len(segment)

            Manager.robot_path.append( best_prepare )
            Manager.robot_path.append( best_move )

            total_object += len(segment)
            total_robot += best_move_size

        # print "Total object:", total_object
        # print "Total robot:", total_robot
        # print "Total:", (total_object + total_robot)

        return total_object, total_robot
