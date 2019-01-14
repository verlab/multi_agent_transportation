#!/usr/bin/python

from thing import Thing

from uuid import uuid4
from collections import namedtuple

class Robot(Thing):

    TYPES = namedtuple("RobotType", "land aerial")(0, 1)

    TYPES_DATA = {
        # Time :: Energy
        0: {'time': 8, 'energy': 1, 'fly': False}, # land
        1: {'time': 1, 'energy': 12, 'fly': True}  # aerial
    }

    def __init__(self, position = (0,0,0), size = (1,1,1), robot_type=TYPES.land, idx = None):
        super(Robot, self).__init__(position, size)

        self.robot_type = robot_type
        self.idx = idx if idx is not None else uuid4().hex

    def copy(self):
        return Robot( self.position, self.size, self.robot_type, self.idx )
