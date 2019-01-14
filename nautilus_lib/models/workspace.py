#!/usr/bin/python

from shapely.geometry import box
from shapely.ops import cascaded_union

import numpy as np

class Workspace(object):

    def __init__(self, size = (10, 10, 10)):
        size = size if type(size) in (tuple, list) else ( (size,) * 3 )

        self._size = np.array( size, np.float32 )
        self._obstacle_list = []
        self.temp_obstacle_list = []

        self._polygon = None
        self._update_polygon()

    def _update_polygon(self):
        self._polygon = box(0, 0, self._size[0], self._size[1])

    @property
    def size(self):
        return self._size

    @size.setter
    def size(self, value):
        self._size = np.array( value, np.float32 )
        self._update_polygon()

    def get_obstacles(self):
        return self._obstacle_list

    def add_obstacle(self, obstacles):
        obstacles = obstacles if type(obstacles) in (tuple, list) else [obstacles]
        self._obstacle_list.extend( obstacles )

    def map_presentation(self):
        return cascaded_union( [obs._polygon for obs in self._obstacle_list] )

    def copy(self):
        other = Workspace( ( self.size[0], self.size[1], 10 ) )

        for obstacle in self._obstacle_list:
            other.add_obstacle( obstacle.copy() )

        return other

    def valid_state(self, thing):
        # Valid the limits of the state
        if not self._polygon.contains( thing._polygon ) or thing.z < 0:
            raise Exception('Invalid state limits!')

        # Valid collision of the state
        if self.test_collision( thing ):
            raise Exception('State collides!')

        return True

    def test_collision(self, thing):
        for obstacle in self._obstacle_list:
            if obstacle.collides( thing ) and not thing.in_air():
                return True

        for obstacle in self.temp_obstacle_list:
            if obstacle.collides( thing ) and not thing.in_air():
                return True

        return False
