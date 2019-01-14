from nautilus_executor.utils import create_environment
from nautilus_executor.manager_single_simple import ManagerSimple

from nautilus_lib import viewer
from nautilus_lib import viewer_animate

import time

# map_path = '/home/ramonmelo/Dropbox/Ramon_Projects/ramon-mestrado/map_creator/maps/map1.json'
map_path = '/home/ramonmelo/Dropbox/Ramon_Projects/ramon-mestrado/map_creator/maps_test/c20.json'

# diff = 8

environment = create_environment( map_path )
# environment['object_list'] = environment['object_list'][0 + diff:3 + diff]

environment['object_list'] = [ environment['object_list'][19], environment['object_list'][0], environment['object_list'][1] ]

print len(environment['object_list'])

# start_time = time.time()

# anim = viewer_animate.ViwerAnimate()
# anim.run()

# ManagerSimple.ani_obj = anim
ManagerSimple.execute( environment )

viewer.view_path_list( environment, ManagerSimple.obj_path, ManagerSimple.robot_path )
# total_time = time.time() - start_time
# print repr(total_time)
