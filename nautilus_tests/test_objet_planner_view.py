#!/usr/bin/python

from nautilus_executor.utils import create_environment
from nautilus_executor.manager import Manager

from nautilus_executor.manager_multi_parallel import ManagerParallel
from nautilus_executor.manager_multi import ManagerHungarianMulti

from nautilus_lib import viewer
from nautilus_lib.viewer import view_path, view_environment

import time
import csv

map_path = '/home/ramonmelo/Dropbox/Ramon_Projects/ramon-mestrado/map_creator/maps_test/visualteste.json'

# map_qt = 1
# obj_qt = [1,5,10,15,20,30,50,100]
# robot_qt = [1]

# cfg = [
#     { 'robot_types': [0,1], 'time_factor': 0.5, 'energy_factor': 0.5 },
#     { 'robot_types': [0,1], 'time_factor': 1, 'energy_factor': 0 },
#     { 'robot_types': [0], 'time_factor': 0.5, 'energy_factor': 0.5 }
# ]

# config = cfg[2]

# time_factor = config['time_factor']
# energy_factor = config['energy_factor']

# config['time_factor'] = float(config['time_factor']) / float( time_factor + energy_factor )
# config['energy_factor'] = float(config['energy_factor']) / float( time_factor + energy_factor )

# environment = create_environment( map_path.format(4, 1) )
environment = create_environment( map_path )

# environment['object_list'] = environment['object_list'][0:3]
# environment['robot_dict'] = { k:v for k, v in environment['robot_dict'].items() if k in environment['robot_dict'].keys()[0:1] }

view_environment( environment )

# Manager._create_object_plan( environment, config )
# ManagerParallel._parallel_object_plan( environment )

# for obj in environment['object_list']:
    # print len(obj['plan']['path'])
#     print obj['plan']['segments']

    # view_path(None, obj['plan']['path'], environment['workspace'])

    # for segment in obj['plan']['segments']:
    #     print segment.graph['start_id']
    #     print segment.graph['end_id']

    #     view_path(None, segment, environment['workspace'])

# start_time = time.time()



# ManagerHungarianMulti.execute( environment )
# viewer.view_path_list( environment, ManagerHungarianMulti.obj_path, ManagerHungarianMulti.robot_path )





# viewer.view_path_list( environment, ManagerSimple.obj_path, ManagerSimple.robot_path )

# total_time = time.time() - start_time
# print repr(total_time)
