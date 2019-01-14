#!/usr/bin/python

import numpy as np
from shapely.geometry import box

class Thing(object):
    def __init__(self, position = (0,0,0), size = (1,1,1)):
        self._size = np.array( size, np.float32 )
        self._position = np.array( position, np.float32 )

        self._polygon = None
        self._update_polygon()

    @property
    def size(self):
        return self._size

    @size.setter
    def size(self, value):
        self._size = np.array( value, np.float32 )
        self._update_polygon()

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, value):
        self._position = np.array( value, np.float32 )
        self._update_polygon()

    @property
    def x(self): return self.position[0]

    @property
    def y(self): return self.position[1]

    @property
    def z(self): return self.position[2]

    @property
    def width(self): return self.size[0]

    @property
    def height(self): return self.size[1]

    @property
    def depth(self): return self.size[2]

    def in_air(self):
        return self.z > (self.depth / 2)

    def _update_polygon(self):
        x, y, z = self._position
        w, h, d = self._size

        x0 = x - (w / 2)
        y0 = y - (h / 2)
        x1 = x + (w / 2)
        y1 = y + (h / 2)

        self._polygon = box(x0, y0, x1, y1)

    def collides(self, other):
        test = self._polygon.crosses( other._polygon ) or self._polygon.equals( other._polygon )

        if (type(other) is Thing) and (type(self) is Thing):
            test = test or self._polygon.touches( other._polygon )

        return test

    def __eq__(self, other):
        if type(self) != type(other): return False
        return self._polygon.equals( other._polygon )

    def __iadd__(self, other):
        self.position += other
        return self

    def __hash__(self):
        return hash(str(self.position))

    def __str__(self):
        return str( self.position )

    def copy(self):
        return Thing( self.position, self.size )
