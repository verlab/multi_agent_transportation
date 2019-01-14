#!/usr/bin/python

from nautilus_executor.utils import create_environment
from nautilus_executor.manager_multi_parallel import ManagerParallel

from nautilus_lib import viewer

import time
import numpy as np

# map_path = '/home/ramonmelo/Dropbox/Ramon_Projects/ramon-mestrado/map_creator/maps/map1.json'
map_path = '/home/ramonmelo/Dropbox/Ramon_Projects/ramon-mestrado/map_creator/maps_test/'
map_name = 'cena_fixa_{}_{}.json'

tipo_mapa = [6]
mapa = [8]

repeticoes = 4

for i in tipo_mapa: # tipo de mapa
    for j in mapa: # mapa

        total_time = []
        temp_map = map_name.format(i, j)

        print 'Processing map', temp_map

        temp_map = map_path + temp_map

        for t in range(0, repeticoes): # repeticoes

            print 'try', t

            environment = create_environment( temp_map )

            environment['object_list'] = environment['object_list'][0:1]
            print len(environment['object_list'])

            start_time = time.time()
            total_object, total_robot = ManagerParallel.execute( environment )
            curr_time = time.time() - start_time

            total_time.append( curr_time )

        median_time = np.sum( total_time ) / repeticoes
        std_time = np.std( total_time )

        print [i, j, total_object, total_robot, median_time, std_time ]

# total_time = []

# temp_map = map_path + map_name.format(5, 1)

# environment = create_environment( temp_map )

# start_time = time.time()
# total_object, total_robot = ManagerParallel.execute( environment )
# curr_time = time.time() - start_time

# total_time.append( curr_time )

# viewer.view_path_list( environment, ManagerSimple.obj_path, ManagerSimple.robot_path )
# viewer.view_path_list( environment, ManagerHungarianMulti.obj_path, ManagerHungarianMulti.robot_path )

# median_time = np.sum( total_time )
# std_time = np.std( total_time )

# print [ total_object, total_robot, median_time, std_time ]
