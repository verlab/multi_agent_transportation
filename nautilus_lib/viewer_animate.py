#!/usr/bin/python
import matplotlib.animation as animation
import matplotlib.pyplot as plt

from shapely.geometry import *
from descartes import PolygonPatch

from multiprocessing import Process, Pipe

import time
import numpy as np
from collections import namedtuple

import networkx as nx

obs_color = namedtuple('ObsColor', 'face edge')('#606060', '#0f0f0f')
item_color = namedtuple('ItemColor', 'face edge')('#abcabc', '#131131')
path_color = namedtuple('PathColor', 'edge start end')('#489fda', '#39db0e', '#db290e')
robot_color = namedtuple('RobotColor', 'face edge')('#e5900f', '#130c01')

color_list = ['#d24a0e', '#ef197f', '#eee410']

class ViwerAnimate(object):
    def __init__(self):
        self.conn1, self.conn2 = Pipe()

        self.update_process = Process(target=self.update, args=(self.conn1,))
        self.show_process = Process(target=self.show, args=(self.conn2,))

        self.data = None
        self.wait = True

    def set_data(self, data):
        robot_path = data['robot_path']

        scene = {
            'robot': None
        }

        position_list = nx.shortest_path( robot_path, robot_path.graph['start_id'], robot_path.graph['end_id'] )

        for position_id in position_list:
            node = robot_path.node[position_id]
            state = node['state']
            x, y, z = state.position

            scene['robot'] = [x, y]

            print 'set scene'
            self.set_scene( scene )

            time.sleep(2)

    def set_scene(self, scene):
        self.scene = scene
        self.wait = False

    def update(self, conn):
        while True:
            while self.wait:
                print 'wait'
                time.sleep(0.5)
            self.wait = True

            scene = self.scene

            robot_poly = PolygonPatch(Point( scene['robot'] ).buffer(0.1))

            patch_data = [robot_poly]

            print 'send'
            conn.send( patch_data )
            time.sleep(0.5)

        conn.send(0)

    def show(self, conn):
        fig = plt.figure(num=1, dpi=100)
        ax = fig.add_subplot(111)
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 10)
        ax.set_aspect(1)

        plt.ion()
        plt.show()
        plt.draw()
        plt.pause(0.1)

        while True:
            data = conn.recv()

            if data == 0: break
            print 'receive data'

            ax.clear()
            for patch in data:
                ax.add_patch( patch )

            plt.draw()
            plt.pause(0.1)

    def run(self):
        self.update_process.start()
        self.show_process.start()

        self.show_process.join()
        self.show_process.join()

        self.conn1.close()
        self.conn2.close()
        print 'end'
