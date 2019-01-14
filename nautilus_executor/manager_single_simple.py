#!/usr/bin/python

from manager import Manager
from nautilus_lib.planner import Planner

class ManagerSimple(Manager):

    # ani_obj = None

    @staticmethod
    def execute( environment ):
        Manager._create_object_plan( environment )

        space = environment['workspace']
        object_list = environment['object_list']
        robot = environment['robot_dict'].values()[0]

        total_object = 0
        total_robot = 0

        # For each object
        for obj in object_list:

            # print "### object", obj['idx']

            # For each segment
            for segment in obj['plan']['segments']:

                segment_start_id = segment.graph['start_id']
                segment_end_id = segment.graph['end_id']

                segment_start = segment.node[ segment_start_id ]
                segment_end = segment.node[ segment_end_id ]

                # print "start"
                # print robot
                # print obj['current_pos']
                # print '---------------'

                # Prepare

                space.temp_obstacle_list = [ item['current_pos'] for item in object_list ]

                robot_end = robot.copy()
                robot_end.position = Planner.create_robot_position( segment_start, 2 ).position

                prepare_plan = Manager._execute_planning({
                    'start_position': robot,
                    'end_position': robot_end,
                    'workspace': space,
                    'robot_type': robot.robot_type
                })

                Manager.robot_path.append( prepare_plan )

                # Execute
                # if ManagerSimple.ani_obj:
                #     data = {
                #         'robot_path': prepare_plan
                #     }
                #     ManagerSimple.ani_obj.set_data( data )

                space.temp_obstacle_list = []

                # Moviment

                robot.position = robot_end.position

                # print "pre move"
                # print robot
                # print obj['current_pos']
                # print '---------------'

                space.temp_obstacle_list = [ item['current_pos'] for item in object_list if item['idx'] is not obj['idx'] ]

                robot_end = robot.copy()
                robot_end.position = Planner.create_robot_position( segment_end ).position

                moviment_plan = Manager._execute_planning({
                    'start_position': robot,
                    'end_position': robot_end,
                    'workspace': space,
                    'robot_type': robot.robot_type
                })

                Manager.robot_path.append( moviment_plan )

                space.temp_obstacle_list = []

                robot.position = robot_end.position
                obj['current_pos'].position = segment_end['state'].position

                # print "move"
                # print robot
                # print obj['current_pos']
                # print '---------------'

                total_object += len( segment )
                total_robot += len(prepare_plan) + len(moviment_plan)

            space.temp_obstacle_list = []

        # print "Total object:", total_object
        # print "Total robot:", total_robot
        # print "Total:", (total_object + total_robot)

        return total_object, total_robot
