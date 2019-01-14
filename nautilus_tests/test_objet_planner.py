#!/usr/bin/python

from nautilus_executor.utils import create_environment
from nautilus_executor.manager import Manager

from nautilus_executor.manager_multi_parallel import ManagerParallel

from nautilus_lib import viewer
from nautilus_lib.viewer import view_path

import time
import csv

map_path = '/home/ramonmelo/Dropbox/Ramon_Projects/ramon-mestrado/map_creator/maps_test/3/1/'
map_name = '1_{}_1.json'

# map_qt = 1
obj_qt = [1,5,10,15,20,30,50,100]
# robot_qt = [1]

cfg = [
    { 'robot_types': [0,1], 'time_factor': 0.5, 'energy_factor': 0.5 },
    { 'robot_types': [0,1], 'time_factor': 1, 'energy_factor': 0 },
    { 'robot_types': [0,1], 'time_factor': 0, 'energy_factor': 1 }
]

# object_data_file = open('object_data.csv', 'wb')
# data_writer = csv.writer(object_data_file)

# for map_index in range(map_qt):
# for robot_qt_index in robot_qt:

for obj_qt_index in obj_qt:

    # Execute test
    temp_map = map_name.format(obj_qt_index)
    temp_map = map_path + temp_map

    print "Processing map", temp_map

    # environment = create_environment( temp_map )
    # config = { 'robot_types': [0], 'time_factor': 0.5, 'energy_factor': 0.5 }

    config = cfg[0]

    total_time = 0
    best_time = None
    wost_time = None

    for i in xrange(0, 5):
        # print 'try', i + 1

        environment = create_environment( temp_map )

        start_time = time.time()
        Manager._create_object_plan( environment, config )
        # ManagerParallel._parallel_object_plan( environment )

        curr_time = time.time() - start_time

        if not best_time or curr_time < best_time:
            best_time = curr_time

        if not wost_time or curr_time > wost_time:
            wost_time = curr_time

        total_time += curr_time

    total_time /= 5

    print [obj_qt_index, total_time, wost_time, best_time]

# data_writer.writerow([map_index, obj_qt_index, robot_qt_index, total_time])
# object_data_file.close()










# for obj in environment['object_list']:
    # print len(obj['plan']['path'])
    # print obj['plan']['segments']

    # for segment in obj['plan']['segments']:
        # print segment.graph['start_id']
        # print segment.graph['end_id']

        # view_path(None, segment, environment['workspace'])

# start_time = time.time()

# ManagerHungarianMulti.execute( environment )

# viewer.view_path_list( environment, ManagerSimple.obj_path, ManagerSimple.robot_path )
# viewer.view_path_list( environment, ManagerHungarianMulti.obj_path, ManagerHungarianMulti.robot_path )

# total_time = time.time() - start_time
# print repr(total_time)
