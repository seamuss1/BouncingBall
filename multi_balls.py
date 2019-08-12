import os, time, datetime, threading, random
import numpy as np
import matplotlib.pyplot as plt
import sys
from cortix.src.module import Module
from cortix.src.port import Port
from cortix.src.cortix_main import Cortix
import time
from plot_multi import Plot
import shapely.geometry as geo
import shapely.ops
from shapely import affinity

class BouncingBall(Module):
    def __init__(self,shape=None, runtime=10, balls=5):
        super().__init__()
        self.shape = shape
        self.runtime = runtime
        self.balls = balls
        self.a = (0,-9.81)
        self.messenger = Messenger()
        
    def run(self):
        t = 0.01
        self.elapsed, oe = 0,0
        self.spawn_balls()
        its = round(self.runtime/t)
        for i in self.ports: #Send initial properties
            if 'plot' not in str(i):
                self.send(self.localballs,i)
        
        for i in self.ports:
            if 'plot' not in str(i):
                messenger = self.recv(i)
                self.balldic[messenger.name] = messenger
        for i in range(its):
            self.elapsed += t
            if oe != int(self.elapsed):
                print('Time Elapsed: ', int(self.elapsed),'seconds')
                oe = int(self.elapsed)
            for line in self.localballs:
                messenger = self.balldic[line]
                name = messenger.name
                p0 = messenger.p
                circle = messenger.circle
                v0 = messenger.vf
                cor = messenger.cor
                m = messenger.m
                r = messenger.r

                #Gravity calculations for timestep
                p0[1] = 0.5*self.a[1]*t**2+v0[1]*t+p0[1]
                p0[0] = 0.5*self.a[0]*t**2+v0[0]*t+p0[0]
                v0[1] = self.a[1]*t + v0[1]
                v0[0] = self.a[0]*t + v0[0]
                #Update position and velocity variables
                pnt = geo.point.Point(p0[0],p0[1])
                circle = pnt.buffer(r)
                self.messenger.v = v0
                for shape in self.bndry: #Detects collision with boundary
                    if circle.crosses(shape) or circle.touches(shape) or circle.intersects(shape):
                        messenger = self.wall_collision(shape, messenger)
                        p0 = messenger.p
                        messenger.vf = messenger.v
                        print(p0,v0)
##                for line in self.balldic: #Detects collision with other objects
##                    messenger2 = self.balldic[line]
##                    shape = self.balldic[line].circle
##                    name = self.balldic[line].name
##                    if messenger.name == line:
##                        continue
##                    for c,line2 in enumerate(self.balldic[line].collision): #Undetected Collisions received as a message
##                        if messenger.name == line2:
##                            messenger = self.ball_collision(messenger,messenger2)
##                            if circle.crosses(shape) or circle.touches(shape) or circle.intersects(shape):
##                                messenger = self.ball_shift(shape,messenger)
##                            del self.balldic[line].collision[c]
##                    #Reacts to intersection between this object and another
##                    if circle.crosses(shape) or circle.touches(shape) or circle.intersects(shape):
##                        messenger = self.ball_collision(messenger,messenger2)
##                        messenger = self.ball_shift(shape, messenger)
##                        messenger.collision.append(name)
                self.localballs[messenger.name] = messenger
            for i in self.ports: #Send and receive messages for each timestep
                self.send(self.localballs,i)
            for name in self.localballs:
                self.localballs[name].collision=[]
            for i in self.ports:
                if 'plot' in str(i): #Not receiving messages from plotting
                    continue
                messenger = self.recv(i)
                self.balldic[messenger.name] = messenger
            
                self.messenger.collision = [] #Reset list of collisions
        for i in self.ports: #Send 'done' string to plot module as end condition
            if 'plot' in str(i):
                self.send('done',i)
    
        print('done')
        return

    def spawn_balls(self):
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
        bn = self.shape.bounds
        self.balldic = dict()
        self.localballs = dict()
        
        for i in range(self.balls):
            name = str(datetime.datetime.now())
            time.sleep(0.1)
            print(name)
            r=1.0
            for i in range(100): #Attempt to spawn ball within boundary
                p0 = [random.uniform(bn[0],bn[2]),random.uniform(bn[1],bn[3])]
                pnt = geo.point.Point(p0[0],p0[1])
                circle = pnt.buffer(r)
                if self.shape.contains(circle):
                    break

            v0 = [random.uniform(-50,50),random.uniform(-30,30)]
            cor = 0.95
            a = (0,-9.81)
            m = 1
            KE = 0.5*m*((v0[0]**2+v0[1]**2)**0.5)**2
            messenger = Messenger()
            messenger.circle = circle
            messenger.name = name
            messenger.m,self.messenger.r = m,r
            messenger.v = v0
            messenger.vf = v0
            messenger.p = p0
            messenger.pnt = pnt
            messenger.cor = cor
            messenger.collision = []
            self.balldic[name] = messenger
            self.localballs[name] = messenger
            
    def wall_collision(self,shape, messenger):
        name = messenger.name
        p0 = messenger.p
        circle = messenger.circle
        v0 = messenger.vf
        cor = messenger.cor
        m = messenger.m
        r = messenger.r
        
        p1,p2 = shapely.ops.nearest_points(circle.centroid,shape)
        angle3 = np.arctan2(p2.y - p1.y, p2.x - p1.x)
        d = shape.distance(circle.centroid)
        p0 = [p0[0]-(r*1.01-d)*np.cos(angle3), p0[1]-(r-d*1.01)*np.sin(angle3)]
        pnt = geo.point.Point(p0[0],p0[1])
        circle = pnt.buffer(r)
        
        angle2 = np.arctan2(v0[1], v0[0])
        v = (v0[0]**2+v0[1]**2)**0.5
        theta = angle2-angle3
        vbi, vbj = v*np.sin(theta), v*np.cos(theta)
        vbj = -vbj *cor
        v = (vbi**2+vbj**2)**0.5
        angle4 = np.arctan2(vbj, vbi)
        angle1 = angle4 - angle3
        
        v0 = [np.sin(angle1)*v, np.cos(angle1)*v]
        messenger.v = v0
        messenger.p = p0
        messenger.circle = circle
        print('wall collision')
        return messenger
    
    def ball_shift(self,shape,messenger):
        name = messenger.name
        p0 = messenger.p
        circle = messenger.circle
        v0 = messenger.v
        cor = messenger.cor
        m = messenger.m
        r = messenger.r
        p1,p2 = shapely.ops.nearest_points(circle.centroid,shape)
        angle = np.arctan2(p2.y - p1.y, p2.x - p1.x)
        d = shape.distance(circle.centroid)
        p0 = [p0[0]-(r*1.01-d)*np.cos(angle),p0[1]-(r*1.01-d)*np.sin(angle)]
        pnt = geo.point.Point(p0[0],p0[1])
        circle = pnt.buffer(r)
        messenger.p = p0
        messenger.circle = circle
        return messenger
    
    def ball_collision(self,messenger,messenger2):
        name = messenger.name
        p0 = messenger.p
        circle = messenger.circle
        v0 = messenger.v
        cor = messenger.cor
        m = messenger.m
        r = messenger.r
        
        shape = messenger2.circle
        v2,m2 = messenger2.v,messenger2.m
        v3 = (v2[0]**2+v2[1]**2)**0.5
        phi = np.arctan2(v2[1],v2[0])
        p1,p2 = shapely.ops.nearest_points(circle.centroid,shape)
        angle = np.arctan2(p2.y - p1.y, p2.x - p1.x) 
        angle2 = np.arctan2(v0[1], v0[0])
        v = (v0[0]**2+v0[1]**2)**0.5
        
        #Equation source: https://en.wikipedia.org/wiki/Elastic_collision
        vpx=((v*np.cos(angle2-angle)*(m-m2)+2*m2*v3*np.cos(phi-angle))/(m+m2))*np.cos(angle)+v*np.sin(angle2-angle)*np.cos(angle+np.pi/2)
        vpy=((v*np.cos(angle2-angle)*(m-m2)+2*m2*v3*np.cos(phi-angle))/(m+m2))*np.sin(angle)+v*np.sin(angle2-angle)*np.sin(angle+np.pi/2)
        vp = (vpx**2+vpy**2)**0.5
        v0 = [vpx,vpy]

        messenger.vf = v0
        messenger.p = p0
        messenger.circle = circle
        print('Ball collision')
        return messenger
        
        
class Messenger:
    def __init__(self, circle=None, timestamp='0'):
        self.circle = circle
        self.collision = []
        self.timestamp = timestamp
        self.name = ''
        self.m = 1
        self.r = 1
        self.v = []
        self.vf = []
        self.p = []
        self.pnt = None
        self.cor = 1

#Example driver script        
if __name__ == '__main__':
    cortix = Cortix(use_mpi=False)
    mod_list = []
    shape = geo.box(-30,0,30,50)
    plot = Plot(shape=shape, modules=1)
    plot.fps = 10
    cortix.add_module(plot)
    for i in range(5):
        app = BouncingBall(shape,runtime=2,balls=10)
        app.r = 0.01
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
    
    
            
