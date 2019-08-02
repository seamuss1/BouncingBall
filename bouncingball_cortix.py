import os, time, datetime, threading, random
import numpy as np
import matplotlib.pyplot as plt
import sys
if "C:/Users/seamu/OneDrive/Documents/GitHub/cortix" not in sys.path:
    sys.path.append("C:/Users/seamu/OneDrive/Documents/GitHub/cortix")
from cortix.src.module import Module
from cortix.src.port import Port
from cortix.src.cortix_main import Cortix
import time
from bb_plot import Plot
import shapely.geometry as geo
import shapely.ops
from shapely import affinity

class BouncingBall(Module):
    def __init__(self,shape=None):
        super().__init__()
        self.shape = shape
        self.bndry = []
        coords = list(self.shape.exterior.coords)
        #Parse the box(LineRing) to create a list of line obstacles
        for c,f in enumerate(coords):
            try:
                cr = geo.LineString([coords[c],coords[c+1]])
            except IndexError:
                cr = geo.LineString([coords[c],coords[-1]])
                break
            
            self.bndry.append(cr)
        self.cor = 0.25
        bn = self.shape.bounds
        self.r=1.0
        for i in range(100):
            self.p0 = [random.uniform(bn[0],bn[2]),random.uniform(bn[1],bn[3])]
            self.pnt = geo.point.Point(self.p0[0],self.p0[1])
            self.circle = self.pnt.buffer(self.r)
            if self.shape.contains(self.circle):
                break
        self.v0 = [random.uniform(-50,50),random.uniform(-30,30)]
        self.cor = 0.95
        self.a = (0,-9.81)
        self.timestamp=str(datetime.datetime.now())

            
    def run(self, time_int=0.01):
        t = time_int
        portdic = dict()
        for i in self.ports:
            if 'plot' not in str(i):
                self.send(self.circle,i)
        
        for i in self.ports:
            if 'plot' not in str(i):
                portdic[str(i)] = self.recv(i)
        for i in range(1000):
            self.p0[1] = 0.5*self.a[1]*t**2+self.v0[1]*t+self.p0[1]
            self.p0[0] = 0.5*self.a[0]*t**2+self.v0[0]*t+self.p0[0]
            self.v0[1] = self.a[1]*t + self.v0[1]
            self.v0[0] = self.a[0]*t + self.v0[0]
            self.pnt = geo.point.Point(self.p0[0],self.p0[1])
            self.circle = self.pnt.buffer(self.r)
            check = False
            while check==False:
                check = True
                for shape in self.bndry:
                    if self.circle.crosses(shape) or self.circle.touches(shape) or self.circle.intersects(shape):
                        self.wall_collision(shape)
                        check = False
                for name in portdic:
                    shape = portdic[name]
                    if self.circle.crosses(shape) or self.circle.touches(shape) or self.circle.intersects(shape):
                        self.ball_collision(shape)
                        check = False
                break

            for i in self.ports:
                self.send(self.circle,i)
            for i in self.ports:
                if 'plot' in str(i):
                    continue
                if str(i) not in portdic:
                    portdic[str(i)] = ''
                circle = self.recv(i)
                portdic[str(i)] = circle
        for i in self.ports:
            if 'plot' in str(i):
                self.send('done',i)
        print('done')
        
        return
    def wall_collision(self,shape):
        pi,pf = list(shape.coords)
        p1,p2 = shapely.ops.nearest_points(self.pnt,shape)
        angle = np.rad2deg(np.arctan2(pf[-1] - pi[-1], pf[0] - pi[0]))
        angle3 = np.arctan2(p2.y - p1.y, p2.x - p1.x)
        d = shape.distance(self.pnt)
        self.p0 = [self.p0[0]-(self.r-d)*np.cos(angle3), self.p0[1]-(self.r-d)*np.sin(angle3)]
        self.circle = self.pnt.buffer(self.r)
        
        angle2 = np.arctan2(self.v0[1], self.v0[0])
        v = (self.v0[0]**2+self.v0[1]**2)**0.5
        theta = angle2-angle3
        vbi, vbj = v*np.sin(theta), v*np.cos(theta)
        vbj = -vbj *self.cor
        v = (vbi**2+vbj**2)**0.5
        angle4 = np.arctan2(vbj, vbi)
        angle1 = angle4 - angle3
        
        self.v0 = [np.sin(angle1)*v, np.cos(angle1)*v]
        print('Wall Collision')
        
    def ball_collision(self,shape):
        [(x1,y1)],[(x2,y2)] = self.pnt.coords, shape.centroid.coords
        p1,p2 = shapely.ops.nearest_points(self.pnt,shape)
        angle = np.arctan2(p2.y - p1.y, p2.x - p1.x)
        d = shape.distance(self.pnt)
        self.p0 = [self.p0[0]-(self.r-d)*np.cos(angle),self.p0[1]-(self.r-d)*np.sin(angle)]
        self.circle = self.pnt.buffer(self.r)
        angle2 = np.arctan2(self.v0[1], self.v0[0])
        theta = angle2-angle
        v = (self.v0[0]**2+self.v0[1]**2)**0.5
        vbi, vbj = v*np.sin(theta), v*np.cos(theta)
        vbj = -vbj *self.cor
        v = (vbi**2+vbj**2)**0.5
        print('Ball collision! Velocity:', v)
        angle4 = np.arctan2(vbj, vbi)
        angle1 =angle4-angle
        
        self.v0 = [np.sin(angle1)*v, np.cos(angle1)*v]
        
if __name__ == '__main__':
    cortix = Cortix(use_mpi=False)
    mod_list = []
    box = geo.box(-30,0,30,50)
    box = affinity.rotate(box,40)
    
    plot = Plot(shape=box)
    cortix.add_module(plot)
    for i in range(20):
        time.sleep(0.01)
        app = BouncingBall(box)
        mod_list.append(app)
        cortix.add_module(app)
                
    for c,i in enumerate(mod_list):
        p1 = Port('plot-send{}'.format(c))
        p3 = Port('plot-receive{}'.format(c))
        i.add_port(p1)
        plot.add_port(p3)
        p1.connect(p3)
        for j in mod_list:
            if i == j:
                continue
            name = '{}{}'.format(i.timestamp,j.timestamp)
            p  = Port(name = '{}{}'.format(i.timestamp,j.timestamp))
            p2 = Port(name = '{}{}'.format(j.timestamp,i.timestamp))
            p.connect(p2)
            i.add_port(p)
            j.add_port(p2)
    cortix.draw_network('network_graph.png')
    cortix.run()
    
            
