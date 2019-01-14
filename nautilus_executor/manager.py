#!/usr/bin/python

import sys
sys.path.append("/home/ramonmelo/Dropbox/Ramon_Projects/ramon-mestrado/")

from nautilus_lib.planner import Planner
from nautilus_lib.analyzer import Analyser

from nautilus_lib.viewer import view_path

class Manager(object):

    robot_path = []
    obj_path = []
    # ani_func = None

    @staticmethod
    def _create_object_plan( environment, config = None ):

        object_list = environment['object_list']
        space = environment['workspace']
        robot_types = list(set([ robot.robot_type for robot in environment['robot_dict'].values() ]))

        print robot_types

        if config:
            robot_types = config['robot_types']

        for obj in object_list:

            space.temp_obstacle_list = []
            space.temp_obstacle_list.extend( [ item['start'] for item in object_list if item['idx'] is not obj['idx'] ] )
            # space.temp_obstacle_list.extend( [ item['end'] for item in object_list if item['idx'] is not obj['idx'] ] )

            test_case = {
                'start_position': obj['start'],
                'end_position': obj['end'],
                'workspace': space,
                'robot_types': robot_types
            }

            try:
                path = Manager._execute_planning( test_case, config )
                segments = Analyser.slipt( path )

                obj['plan'] = {
                    'path': path,
                    'segments': segments,
                    'total': len(path)
                }

                Manager.obj_path.append( path )

            except Exception, e:
                print "Object: ", obj['idx'] + 1
                print e.message

            space.temp_obstacle_list = []

        print 'finish object planning'

    @staticmethod
    def _execute_planning( test_case, config = None ):

        planner = Planner( config = config )
        plan = planner.plan( test_case )

        # view_path( planner, plan )

        return plan
