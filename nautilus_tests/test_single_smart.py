from nautilus_executor.utils import create_environment
from nautilus_executor.manager_single_smart import ManagerSmart

from nautilus_lib import viewer
from nautilus_lib.viewer import view_path, view_environment

import time

# map_path = '/home/ramonmelo/Dropbox/Ramon_Projects/ramon-mestrado/impl/src/system/system_library/src/map_creator/maps/map1.json'

map_path = '/home/ramonmelo/Dropbox/Ramon_Projects/ramon-mestrado/map_creator/maps_test/c20.json'

# diff = 8

environment = create_environment( map_path )
# environment['object_list'] = environment['object_list'][0 + diff:3 + diff]

environment['object_list'] = [ environment['object_list'][19], environment['object_list'][0], environment['object_list'][1] ]

print len(environment['object_list'])

# view_environment( environment )

# total_time = 0

# for i in xrange(0, 1):

#     environment = create_environment( map_path )

#     start_time = time.time()
#     total_object, total_robot = ManagerSmart.execute( environment )
#     total_time += time.time() - start_time

# total_time /= 5

# print [ total_object, total_robot, total_time ]

ManagerSmart.execute( environment )

viewer.view_path_list( environment, ManagerSmart.obj_path, ManagerSmart.robot_path )

# print repr(total_time)
