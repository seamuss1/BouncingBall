import os, time, datetime, threading, random
import numpy as np
import matplotlib.pyplot as plt
import sys
from cortix.src.module import Module
from cortix.src.port import Port
from cortix.src.cortix_main import Cortix
import time
from bb_plot import Plot
import shapely.geometry as geo
import shapely.ops
from shapely import affinity

class BouncingBall(Module):
    def __init__(self,shape=None, runtime=10):
        super().__init__()
        self.shape = shape
        self.runtime = runtime
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
        self.m = 1
        self.KE = 0.5*self.m*((self.v0[0]**2+self.v0[1]**2)**0.5)**2
        self.timestamp=str(datetime.datetime.now())
        self.messenger = Messenger()
        self.messenger.circle = self.circle
        self.messenger.timestamp = self.timestamp
        self.messenger.m,self.messenger.r = 1,1
        self.messenger.v = self.v0
        self.messenger.p = self.p0
        
    def run(self, state_comm=None, idx_comm=None):
        state_comm.put((idx_comm,self.state))
        t = 0.01
        its = round(self.runtime/t)
        portdic = dict()
        for i in self.ports:
            if 'plot' not in str(i):
                self.send(self.messenger,i)
        
        for i in self.ports:
            if 'plot' not in str(i):
                portdic[str(i)] = self.recv(i)
        for i in range(its):
            self.p0[1] = 0.5*self.a[1]*t**2+self.v0[1]*t+self.p0[1]
            self.p0[0] = 0.5*self.a[0]*t**2+self.v0[0]*t+self.p0[0]
            self.v0[1] = self.a[1]*t + self.v0[1]
            self.v0[0] = self.a[0]*t + self.v0[0]
            self.pnt = geo.point.Point(self.p0[0],self.p0[1])
            self.circle = self.pnt.buffer(self.r)
            
            for shape in self.bndry:
                if self.circle.crosses(shape) or self.circle.touches(shape) or self.circle.intersects(shape):
                    self.wall_collision(shape)
            for name in portdic:
                messenger = portdic[name]
                shape = portdic[name].circle
                ts = portdic[name].timestamp
                if self.circle.crosses(shape) or self.circle.touches(shape) or self.circle.intersects(shape):
                    self.ball_collision(messenger)
                    #self.ball_shift(shape)
                    self.messenger.collision.append(ts)
                    continue
                
                for line in portdic[name].collision:

                    if self.timestamp == line:
                        self.ball_collision(messenger)
                        
            self.messenger.circle = self.circle
            self.messenger.v = self.v0
            self.messenger.p = self.p0
            for i in self.ports:
                self.send(self.messenger,i)
            for i in self.ports:
                if 'plot' in str(i):
                    continue
                messenger = self.recv(i)
                portdic[str(i)] = messenger
            
            self.messenger.collision = [] #Reset list of collisions
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

    def ball_shift(self,shape):
        p1,p2 = shapely.ops.nearest_points(self.pnt,shape)
        angle = np.arctan2(p2.y - p1.y, p2.x - p1.x)
        d = shape.distance(self.pnt)
        self.p0 = [self.p0[0]-(self.r*1.05-d)*np.cos(angle),self.p0[1]-(self.r*1.05-d)*np.sin(angle)]
        self.pnt = geo.point.Point(self.p0[0],self.p0[1])
        self.circle = self.pnt.buffer(self.r)
        
    def ball_collision(self,messenger):
        shape = messenger.circle
        v2,m = messenger.v,messenger.m
        v3 = (v2[0]**2+v2[1]**2)**0.5
        phi = np.arctan2(v2[1],v2[0])
        p1,p2 = shapely.ops.nearest_points(self.pnt,shape)
        angle = np.arctan2(p2.y - p1.y, p2.x - p1.x)        
        angle2 = np.arctan2(self.v0[1], self.v0[0])
        v = (self.v0[0]**2+self.v0[1]**2)**0.5
        
        #Equation source: https://en.wikipedia.org/wiki/Elastic_collision
        vpx=((v*np.cos(angle2- angle)*(self.m-m)+2*m*v3*np.cos(phi-angle))/(self.m+m))*np.cos(angle)+v*np.sin(angle2-angle)*np.cos(angle+np.pi/2)
        vpy=((v*np.cos(angle2- angle)*(self.m-m)+2*m*v3*np.cos(phi-angle))/(self.m+m))*np.sin(angle)+v*np.sin(angle2-angle)*np.cos(angle+np.pi/2)
        
        self.v0 = [vpx,vpy]

class Messenger:
    def __init__(self, circle=None, collision = [], timestamp='0'):
        self.circle = circle
        self.collision = collision
        self.timestamp = timestamp
        self.m = 1
        self.r = 1
        self.v = []
        self.p = []
        
if __name__ == '__main__':
    cortix = Cortix(use_mpi=False)
    mod_list = []
    shapes = ['triangle', 'squares', 'diamond']
    while True:
        print('Choose a shape: 1) Triangle, 2) Square, or 3) Diamond\n')
        shape = input('>>>')
        shape = shape.lower()
        if shape == 'triangle' or shape =='1':
            shape = geo.Polygon([(0, 0), (0, 60), (30, 30)])
            break
        if shape == 'square' or shape =='2':
            shape = geo.box(-30,0,30,50)
            break
        if shape == 'triangle' or shape =='3':
            shape = geo.box(-30,0,30,50)
            shape = affinity.rotate(shape,45)
            break
        print('Input not recognized, try again')
        
    while True:
        print('Choose the number of Bouncing Balls\n')
        balls = input('>>>')
        try:
            balls = int(balls)
            if balls > 1000:
                print('Wow good luck')
            elif balls > 0:
                break
            else:
                print('Choose a better number')
        except:
            print('Entry invalid')
    while True:
        print('How many seconds is the simulation?\n')
        secs = input('>>>')
        try:
            secs = int(secs)
            if secs > 50000:
                print('Wow good luck')
            elif secs > 0:
                break
            else:
                print('Choose a better number')
        except:
            print('Entry invalid')
    plot = Plot(shape=shape, length=balls)
    cortix.add_module(plot)
    for i in range(balls):
        time.sleep(0.01)
        app = BouncingBall(shape,runtime=secs)
        mod_list.append(app)
        cortix.add_module(app)
                
    for c,i in enumerate(mod_list):
        i.connect('plot-send{}'.format(c),plot.get_port('plot-receive{}'.format(c)))
        for j in mod_list:
            if i == j:
                continue
            name = '{}{}'.format(i.timestamp,j.timestamp)
            name2 = '{}{}'.format(j.timestamp,i.timestamp)
            j.connect(name, i.get_port(name2))
            
    cortix.draw_network('network_graph.png')
    cortix.run()
    print('bye')
    
    
            
