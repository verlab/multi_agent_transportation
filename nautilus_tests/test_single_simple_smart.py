from nautilus_executor.utils import create_environment

from nautilus_executor.manager_single_smart import ManagerSmart
from nautilus_executor.manager_single_simple import ManagerSimple

from nautilus_lib import viewer

import time

import numpy as np
import csv

simple_smart_file = open('simple_smart2.csv', 'wb')
data_writer = csv.writer( simple_smart_file )

# map_path = '/home/ramonmelo/Dropbox/Ramon_Projects/ramon-mestrado/impl/src/system/system_library/src/map_creator/maps/map1.json'

# map_path = '/home/ramonmelo/Dropbox/Ramon_Projects/ramon-mestrado/map_creator/maps_test/comp{}.json'
map_path = '/home/ramonmelo/Dropbox/Ramon_Projects/ramon-mestrado/map_creator/maps_test/c{}.json'

ms = [1,5,10,15,20]

for m in ms:

    best_smart = None
    wost_smart = None

    best_simple = None
    wost_simple = None

    total_time_simple = 0
    total_time_smart = 0

    time_smart = []
    time_simple = []

    print 'map', m

    for i in range(0, 5):

        # Smart
        environment = create_environment( map_path.format(m) )

        start_time = time.time()
        t_object_smart, t_robot_smart = ManagerSmart.execute( environment )
        curr_time = time.time() - start_time
        total_time_smart += curr_time
        time_smart.append( curr_time )

        if not best_smart or best_smart > curr_time:
            best_smart = curr_time

        if not wost_smart or wost_smart < curr_time:
            wost_smart = curr_time

        print '-- smart'

        # Simple
        environment = create_environment( map_path.format(m) )

        start_time = time.time()
        t_object_simple, t_robot_simple = ManagerSimple.execute( environment )
        curr_time = time.time() - start_time
        total_time_simple += curr_time
        time_simple.append( curr_time )

        if not best_simple or best_simple > curr_time:
            best_simple = curr_time

        if not wost_simple or wost_simple < curr_time:
            wost_simple = curr_time

        print '-- simple'

    total_time_smart /= 5.0
    total_time_simple /= 5.0

    time_smart = np.std( time_smart )
    time_simple = np.std( time_simple )

    data = [m, t_object_smart, t_robot_smart, total_time_smart, time_smart, best_smart, wost_smart, t_object_simple, t_robot_simple, total_time_simple, time_simple, best_simple, wost_simple ]

    print data
    data_writer.writerow( data )

simple_smart_file.close()

# viewer.view_path_list( environment, ManagerSmart.obj_path, ManagerSmart.robot_path )
# print repr(total_time)
