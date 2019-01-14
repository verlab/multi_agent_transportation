#!/usr/bin/python

from nautilus_executor.utils import create_environment
from nautilus_executor.manager_multi import ManagerHungarianMulti
from nautilus_executor.manager_multi_parallel import ManagerParallel

from nautilus_lib import viewer

import time
import csv

# object_data_file = open('robot_data2.csv', 'a')
# data_writer = csv.writer(object_data_file)

# map_path = '/home/ramonmelo/Dropbox/Ramon_Projects/ramon-mestrado/map_creator/maps_test/0/1/0_1_1.json'

# map_path = '/home/ramonmelo/Dropbox/Ramon_Projects/ramon-mestrado/map_creator/maps_test/1/{0}/1_{1}_{0}.json'

# robot_qt = [1, 3, 5] # 1, 3, 5, 10, 12, 15, 20
# obj_qt = [1,5] #1,5,10,15,20,30,50,100 99

# for o in obj_qt:
#     for r in robot_qt:

#         map_file = map_path.format(r,o)

#         total_time_parallel = 0
#         total_time_serial = 0

#         best_parallel = None
#         wost_parallel = None

#         best_serial = None
#         wost_serial = None

#         for i in xrange(0,3):

#             # Serial
#             environment = create_environment( map_file )

#             start_time = time.time()
#             total_object, total_robot = ManagerHungarianMulti.execute( environment )
#             curr_time = time.time() - start_time

#             total_time_serial += curr_time

#             if not best_serial or best_serial > curr_time:
#                 best_serial = curr_time

#             if not wost_serial or wost_serial < curr_time:
#                 wost_serial = curr_time

#             # Parallel
#             environment = create_environment( map_file )

#             start_time = time.time()
#             total_object, total_robot = ManagerParallel.execute( environment )
#             curr_time = time.time() - start_time

#             total_time_parallel += curr_time

#             if not best_parallel or best_parallel > curr_time:
#                 best_parallel = curr_time

#             if not wost_parallel or wost_parallel < curr_time:
#                 wost_parallel = curr_time


#         total_time_serial /= 3
#         total_time_parallel /= 3

#         data = [o,r,total_object,total_robot, total_time_serial, wost_serial, best_serial, total_time_parallel, wost_parallel, best_parallel]
#         print data
#         data_writer.writerow(data)

# object_data_file.close()

map_path = '/home/ramonmelo/Dropbox/Ramon_Projects/ramon-mestrado/map_creator/maps_test/c10.json'

# map_path = '/home/ramonmelo/Dropbox/Ramon_Projects/ramon-mestrado/map_creator/maps_test/0_5_1.json'

environment = create_environment( map_path )
# start_time = time.time()
ManagerHungarianMulti.execute( environment )

# Total object: 89
# Total robot: 182
# Total: 271


# viewer.view_path_list( environment, ManagerSimple.obj_path, ManagerSimple.robot_path )
viewer.view_path_list( environment, ManagerHungarianMulti.obj_path, ManagerHungarianMulti.robot_path )

# total_time = time.time() - start_time
# print repr(total_time)
