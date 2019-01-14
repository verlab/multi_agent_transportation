#!/usr/bin/python

import simplejson as json
from os import listdir
from os.path import isfile, join
import copy

# rbt_n = [1,3,5,10,15,20]

file_path = '/home/ramonmelo/Dropbox/Ramon_Projects/ramon-mestrado/map_creator/maps_test/1/{}/'
file_map = '/home/ramonmelo/Dropbox/Ramon_Projects/ramon-mestrado/map_creator/maps_test/1/1_100_20.json'
file_map_template = '1_{}_{}.json'

config = json.load( open( file_map ) )

# print 'obj', len(config['object_list'])
# print 'rbt', len(config['robot_list'])

new_config_objects = [
    config['object_list'][0:100],
    config['object_list'][0:50],
    config['object_list'][0:30],
    config['object_list'][0:20],
    config['object_list'][0:15],
    config['object_list'][0:10],
    config['object_list'][0:5],
    config['object_list'][0:1]
]

new_config_robots = [
    config['robot_list'][0:20],
    config['robot_list'][0:15],
    config['robot_list'][0:12],
    config['robot_list'][0:10],
    config['robot_list'][0:5],
    config['robot_list'][0:3],
    config['robot_list'][0:1]
]

for config_robot in new_config_robots:

    for config_object in new_config_objects:

        new_config = copy.deepcopy( config )

        new_config['object_list'] = config_object
        new_config['robot_list'] = config_robot

        new_config = json.dumps( new_config )

        new_file_map = file_map_template.format( len(config_object), len(config_robot) )

        f = open( file_path.format(len(config_robot)) + new_file_map, 'w' )
        f.write(new_config)
        f.close()

# for news in new_files:
#     new_config = copy.deepcopy( config )
#     new_config['object_list'] = news

#     new_file_map = file_map_template.format( len(news) )

#     new_config = json.dumps( new_config )

#     f = open( file_path + new_file_map, 'w' )
#     f.write(new_config)
#     f.close()
