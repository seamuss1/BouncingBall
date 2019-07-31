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
    def __init__(self,bndry=None):
        super().__init__()
        self.bndry = bndry
        self.cor = 0.95
        self.p0 = [random.randint(0,10),random.randint(10,30)]
        self.r=1.0
        self.v0 = [random.uniform(-50,50),random.uniform(-10,10)]
        self.cor = 0.95
        self.a = (0,-9.81)
        self.timestamp=str(datetime.datetime.now())
##        self.box = geo.box(-30,0,30,50)
##        self.box = affinity.rotate(self.box,40)
##        self.bndry = []
##        c = 0
##        coords = list(self.box.exterior.coords)
##        #Parse the box(LineRing) to create a list of line obstacles
##        for f in coords:
##            try:
##                cr = geo.LineString([coords[c],coords[c+1]])
##            except IndexError:
##                cr = geo.LineString([coords[c],coords[-1]])
##                break
##            
##            self.bndry.append(cr)
##            c +=1
            
    def run(self, time_int=0.01):
        t = time_int
        portdic = dict()
        for i in range(300):
            self.p0[1] = 0.5*self.a[1]*t**2+self.v0[1]*t+self.p0[1]
            self.p0[0] = 0.5*self.a[0]*t**2+self.v0[0]*t+self.p0[0]
            self.v0[1] = self.a[1]*t + self.v0[1]
            self.v0[0] = self.a[0]*t + self.v0[0]
            self.pnt = geo.point.Point(self.p0[0],self.p0[1])
            self.circle = self.pnt.buffer(self.r)
            for shape in self.bndry:
                if self.circle.crosses(shape) or self.circle.touches(shape) or self.circle.intersects(shape):
                    p1,p2 = shapely.ops.nearest_points(shape,self.pnt)
                    self.wall_collision(shape,p1,p2)
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
    def wall_collision(self,shape,p1,p2):
        pi,pf = list(shape.coords)
        angle = np.rad2deg(np.arctan2(pf[-1] - pi[-1], pf[0] - pi[0]))
        angle2 = np.rad2deg(np.arctan2(self.v0[1], self.v0[0]))
        v = (self.v0[0]**2+self.v0[1]**2)**0.5
        print('vi',v)
        theta = angle - angle2
        vbi, vbj = v*np.sin(np.deg2rad(theta)), v*np.cos(np.deg2rad(theta))
        vbj = -vbj *self.cor
        v = (vbi**2+vbj**2)**0.5
        angle2 = np.rad2deg(np.arctan2(vbj, vbi))
        angle1 =angle+angle2
        
        self.v0 = [-np.sin(np.deg2rad(angle1))*v, np.cos(np.deg2rad(angle1))*v]
        print(pi,pf)
        print(angle2, 'vf',v)
        print(angle)
        print('angle1', angle1)
        print(self.v0)


if __name__ == '__main__':
    cortix = Cortix(use_mpi=False)
    mod_list = []
    box = geo.box(-30,0,30,50)
    box = affinity.rotate(box,40)
    bndry = []
    c = 0
    coords = list(box.exterior.coords)
    #Parse the box(LineRing) to create a list of line obstacles
    for f in coords:
        try:
            cr = geo.LineString([coords[c],coords[c+1]])
        except IndexError:
            cr = geo.LineString([coords[c],coords[-1]])
            break
        
        bndry.append(cr)
        c +=1
    plot = Plot(bndry)
    cortix.add_module(plot)
    for i in range(3):
        time.sleep(0.01)
        app = BouncingBall(bndry)
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
    cortix.run()
    
            
