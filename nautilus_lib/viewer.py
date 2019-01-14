#!/usr/bin/python

from matplotlib import pyplot
from shapely.geometry import *
from shapely.affinity import translate

from descartes import PolygonPatch

from collections import namedtuple

import numpy as np

obs_color = namedtuple('ObsColor', 'face edge')('#606060', '#0f0f0f')
item_color = namedtuple('ItemColor', 'face edge')('#abcabc', '#131131')
item_color_end = namedtuple('ItemColorEnd', 'face edge')('#ff2c1e', '#131131')

path_color = namedtuple('PathColor', 'edge start end')('#489fda', '#39db0e', '#db290e')
robot_color = namedtuple('RobotColor', 'face edge')('#e5900f', '#130c01')

color_list = ['#d24a0e', '#ef197f', '#eee410', '#0f92e5', '#650edb', '']

color_path = ['#ee8210']

import random
r = lambda: random.randint(0,255)
rx = lambda: '#%02X%02X%02X' % (r(),r(),r())

# print('#%02X%02X%02X' % (r(),r(),r()))

def get_point_list( path ):
    point_list = []

    start_node = path.node[ path.graph['start_id'] ]
    end_node = path.node[ path.graph['end_id'] ]

    current_id = start_node['id']

    while True:
        node = path.node[ current_id ]
        neighbor = path.neighbors( current_id )

        x, y = node['state'].x, node['state'].y
        point_list.append( Point(x, y) )

        if len(neighbor) == 0: break

        current_id = neighbor[0]

    return point_list, start_node, end_node

def view_path_list( environment, path_list_objects, path_list_robot ):

    space = environment['workspace']

    fig = pyplot.figure(num=1, dpi=100)
    ax = fig.add_subplot(111)

    robot_point_list = []
    robot_start = None

    for path in path_list_robot:
        point_list, start_node, end_node = get_point_list( path )

        # if robot_start == None:
            # robot_start = start_node

        # robot_point_list.extend( point_list )

        line = LineString( point_list )
        line = line.buffer(0.25)

        patch = PolygonPatch( line, edgecolor=rx(), alpha=0.3 )
        ax.add_patch(patch)

        robot_start_point = Point( start_node['state'].x, start_node['state'].y ).buffer(0.4)
        patch_start_point = PolygonPatch( robot_start_point, facecolor=robot_color.face, edgecolor=robot_color.edge )

        # ax.add_patch(patch_start_point)

    counter = 1

    for path in path_list_objects:
        break
        point_list, start_node, end_node = get_point_list( path )

        if len(point_list) == 1:
            continue

        line = LineString( point_list )
        line = line.buffer(0.01)

        # color_list.pop(0)
        patch = PolygonPatch( line, edgecolor=color_path[0] )
        ax.add_patch(patch)

        start_point = Point( start_node['state'].x, start_node['state'].y ).buffer(0.4)
        end_point = Point( end_node['state'].x, end_node['state'].y ).buffer(0.4)

        patch_start_point = PolygonPatch( start_point, facecolor=path_color.start, edgecolor=path_color.start )
        ax.add_patch(patch_start_point)

        ax.text(start_node['state'].x + 0.5, start_node['state'].y - 0.5, str(counter), fontsize=15)

        patch_end_point = PolygonPatch( end_point, facecolor=path_color.end, edgecolor=path_color.end )
        ax.add_patch(patch_end_point)

        ax.text(end_node['state'].x + 0.5, end_node['state'].y - 0.5, str(counter), fontsize=15)

        counter += 1

    for obs in space._obstacle_list:

        patch = PolygonPatch( obs._polygon, facecolor=obs_color.face, edgecolor=obs_color.edge, alpha=0.3 )
        ax.add_patch( patch )


    ax.set_xlim(-1, space.size[0] + 1)
    ax.set_ylim(-1, space.size[1] + 1)

    ax.set_xticks(np.arange(0.5,space.size[0] + 0.5,1))
    ax.set_yticks(np.arange(0.5,space.size[1] + 0.5,1))
    # ax.set_xticks(np.arange(0,space.size[0],1))
    # ax.set_yticks(np.arange(0,space.size[1],1))
    # ax.set_xlim(19, 62)
    # ax.set_ylim(2, 32)

    ax.set_xlim(18, 63)
    ax.set_ylim(2, 38)

    ax.set_aspect(1)

    pyplot.grid()
    pyplot.show()

def view_path(planner, path = None, workspace = None):
    point_list = []
    space = workspace if workspace else planner._test_case['workspace']
    # current_id = hash( planner._test_case['start_position'] )

    fig = pyplot.figure(num=1, dpi=100)
    ax = fig.add_subplot(111)

    start_node = path.node[ path.graph['start_id'] ]
    end_node = path.node[ path.graph['end_id'] ]

    current_id = start_node['id']

    while True:
        node = path.node[ current_id ]
        neighbor = path.neighbors( current_id )

        z = 1 if node['state'].in_air() else 0

        x, y = node['state'].x, node['state'].y
        point_list.append( Point(x, y, z) )

        if len(neighbor) == 0: break

        current_id = neighbor[0]

    if len(point_list) == 1:
        return

    # line = LineString( point_list )
    # line = line.buffer(0.01)

    start_point = Point( start_node['state'].x, start_node['state'].y ).buffer(0.5)
    end_point = Point( end_node['state'].x, end_node['state'].y ).buffer(0.5)

    # patch = PolygonPatch( line, edgecolor=path_color.edge )
    # ax.add_patch(patch)

    patch_start_point = PolygonPatch( start_point, facecolor=path_color.start, edgecolor=path_color.start )
    patch_end_point = PolygonPatch( end_point, facecolor=path_color.end, edgecolor=path_color.end )

    sub_point_list = []
    last = None
    pl = []

    for p in point_list:
        c = '#a6a339' if p.z == 1 else '#3e3cae'

        if not last:
            last = c

        if last != c:
            last = c

            sub_point_list.append( pl )
            pl = []

        pl.append(p)
    sub_point_list.append( pl )

    for pl in sub_point_list:
        if not pl:
            continue

        c = '#f45700' if pl[0].z == 1 else '#3e3cae'

        # '#f45700'

        if len(pl) > 1:
            line = LineString( pl )
            line = line.buffer(0.4)

            patch_line = PolygonPatch( line, facecolor=c, edgecolor=c, alpha=0.8 )
            ax.add_patch( patch_line )
        else:
            patch_point = PolygonPatch( pl[0].buffer(0.4), facecolor=c, edgecolor=c, alpha=0.8 )
            ax.add_patch( patch_point )

    ax.add_patch(patch_start_point)
    ax.add_patch(patch_end_point)

    for obs in space._obstacle_list:
        patch = PolygonPatch( obs._polygon, facecolor=obs_color.face, edgecolor=obs_color.edge, alpha=0.3 )
        ax.add_patch( patch )

    ax.set_xlim(-1, space.size[0] + 1)
    ax.set_ylim(-1, space.size[1] + 1)
    # ax.set_xlim(0, 15)
    # ax.set_ylim(0, 15)
    ax.set_aspect(1)

    pyplot.show()

def view_environment( environment ):
    space = environment['workspace']
    object_list = environment['object_list']
    robot_dict = environment['robot_dict']

    fig = pyplot.figure(num=1, dpi=100)
    ax = fig.add_subplot(111)

    trans_x = -3.5
    trans_y = -0.5

    for obs in space._obstacle_list:

        poly = translate( obs._polygon, trans_x, trans_y )

        patch = PolygonPatch( poly, facecolor=obs_color.face, edgecolor=obs_color.edge )
        ax.add_patch( patch )

    for obj in object_list:
        start = obj['start']
        end = obj['end']

        ax.text(start.x + trans_x, start.y - 0.05 + trans_y, str( obj['idx'] + 1 ), fontsize=10, va='center', ha='center')
        ax.text(end.x + trans_x, end.y - 0.05 + trans_y, str( obj['idx'] + 1 ), fontsize=10, va='center', ha='center')

        poly_start = translate( start._polygon, trans_x, trans_y )
        poly_end = translate( end._polygon, trans_x, trans_y )

        start_patch = PolygonPatch( poly_start, facecolor=item_color.face )
        end_patch = PolygonPatch( poly_end, facecolor=item_color_end.face )

        ax.add_patch( start_patch )
        ax.add_patch( end_patch )

    for rob in robot_dict:
        robot = robot_dict[rob]

        if robot.robot_type == 0:
            p = Point( robot.x + trans_x, robot.y + trans_y )
            patch = PolygonPatch( p.buffer(0.5), facecolor=robot_color.face, edgecolor=robot_color.edge )

        else:
            p1 = (robot.x + trans_x + 0.45, robot.y + trans_y - 0.45)
            p2 = (robot.x + trans_x - 0.45, robot.y + trans_y - 0.45)
            p3 = (robot.x + trans_x, robot.y + trans_y + 0.45)

            poly = Polygon([p1, p2, p3])

            patch = PolygonPatch( poly, facecolor=robot_color.face, edgecolor=robot_color.edge )

        # robot._polygon

        ax.add_patch( patch )

    # ax.set_xlim(-1, space.size[0] + 1)
    # ax.set_ylim(-1, space.size[1] + 1)

    ax.set_xticks(np.arange(0, space.size[0] + 0,1))
    ax.set_yticks(np.arange(0, space.size[1] + 0,1))

    ax.set_xlim(0, 14)
    ax.set_ylim(0, 11)

    ax.set_aspect(1)
    # pyplot.gca().invert_xaxis()
    pyplot.grid()
    pyplot.show()

def view_test_case(test_case):
    space = test_case["workspace"]
    start = test_case["start_position"]
    end = test_case["end_position"]

    fig = pyplot.figure(num=1, dpi=100)
    ax = fig.add_subplot(111)

    for obs in space._obstacle_list:
        patch = PolygonPatch( obs._polygon, facecolor=obs_color.face, edgecolor=obs_color.edge )
        ax.add_patch( patch )

    start_patch = PolygonPatch( start._polygon, facecolor=item_color.face )
    end_patch = PolygonPatch( end._polygon, facecolor=item_color.face )

    ax.add_patch( start_patch )
    ax.add_patch( end_patch )

    ax.set_xlim(-1, space.size[0] + 1)
    ax.set_ylim(-1, space.size[1] + 1)
    ax.set_aspect(1)

    pyplot.show()

def view_map(m):
    fig = pyplot.figure(num=1, dpi=100)
    ax = fig.add_subplot(111)

    for obs in m._obstacle_list:
        patch = PolygonPatch( obs._polygon, facecolor=obs_color.face, edgecolor=obs_color.edge )
        ax.add_patch( patch )

    ax.set_xlim(-1, m.size[0] + 1)
    ax.set_ylim(-1, m.size[1] + 1)
    ax.set_aspect(1)
    pyplot.show()
