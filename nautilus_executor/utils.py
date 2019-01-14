#!/usr/bin/python

import simplejson as json

from nautilus_lib.models.workspace import Workspace
from nautilus_lib.models.thing import Thing
from nautilus_lib.models.robot import Robot

def create_environment( file_path ):
    config = json.load( open( file_path ) )

    x, y = config['size'][0], config['size'][1]

    workspace = Workspace( ( int(float(x)), int(float(y)), 10 ) )

    if 'obstacle_list' in config:
        for obs in config['obstacle_list']:

            x = int( float( obs['x'] ))
            y = int( float( obs['y'] ))
            w = int( float( obs['width'] ))
            h = int( float( obs['height'] ))

            obstacle = Thing(
                position = ( x, y, 0.5),
                size =     ( w, h, 1 )
            )

            workspace.add_obstacle( obstacle )

    object_list = []

    if 'object_list' in config:
        for obj in config['object_list']:

            idx = int( obj['id'] )
            w = int( float( obj['width'] ))
            h = int( float( obj['height'] ))

            x_start = int( float( obj['start']['x'] ))
            y_start = int( float( obj['start']['y'] ))

            x_end = int( float( obj['end']['x'] ))
            y_end = int( float( obj['end']['y'] ))

            start = Thing(
                position = ( x_start, y_start, 0.5 ),
                size =     ( w, h, 1 )
            )

            end = Thing(
                position = ( x_end, y_end, 0.5 ),
                size =     ( w, h, 1 )
            )

            object_list.append({
                'idx': idx,
                'current_pos': start,
                'start': start,
                'end': end
            })

    robot_dict = {}

    if 'robot_list' in config:
        for idx, rob in enumerate(config['robot_list']):

            robot_type = int( rob['type'] )
            x = int( float( rob['x'] ))
            y = int( float( rob['y'] ))
            w = int( float( rob['width'] ))
            h = int( float( rob['height'] ))

            robot = Robot(
                position = ( x, y, 0.5 ),
                size = ( w, h, 1 ),
                robot_type = robot_type,
                idx=idx
            )

            robot_dict[ robot.idx ] = robot

    return {
        'workspace': workspace,
        'object_list': object_list,
        'robot_dict': robot_dict
    }
