import numpy as np
import matplotlib.animation as animation
import matplotlib.pyplot as plt

from shapely.geometry import *
from descartes import PolygonPatch

from multiprocessing import Process, Pipe
import time

class Viwer(object):
    def __init__(self):
        self.conn1, self.conn2 = Pipe()

        self.update_process = Process(target=self.update, args=(self.conn1,))
        self.show_process = Process(target=self.show, args=(self.conn2,))

        self.points = [[1,1], [2,2], [3,3], [4,4], [5,5]]

    def get_data(self):
        return self.points.pop()

    def update(self, conn):
        points = ([1,1], [2,2], [3,3], [4,4], [5,5])
        time.sleep(2)

        while True:
            try:
                data_point = self.get_data()
            except IndexError, e:
                break

            poly = PolygonPatch(Point( data_point ).buffer(0.1))
            conn.send( poly )
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
            ax.add_patch( data )
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

if __name__ == '__main__':
    Viwer().run()
