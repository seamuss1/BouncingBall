import os, time, datetime, threading, random
import numpy as np
import matplotlib.pyplot as plt
from cortix.src.module import Module
import time 
import shapely.geometry as geo
import shapely.ops
from shapely import affinity

class BouncingBall:#(Module):
    def __init__(self):
        super().__init__()
        self.cor = 0.95
        self.p0 = [random.randint(0,10),random.randint(10,30)]
        self.r=1.0
        self.pnt = geo.point.Point(self.p0[0],self.p0[1])
        self.circle = self.pnt.buffer(self.r)
        self.v0 = [random.uniform(0,70),random.uniform(-5,5)]
        self.cor = 0.95
        self.a = (0,-9.81)
        self.timestamp=str(datetime.datetime.now())
        self.box = geo.box(-30,0,30,50)
        self.bndry = []
        c = 0
        coords = list(self.box.exterior.coords)
        #Parse the box(LineRing) to create a list of line obstacles
        for f in coords:
            try:
                cr = geo.LineString([coords[c],coords[c+1]])
            except IndexError:
                cr = geo.LineString([coords[c],coords[-1]])
                break
            
            self.bndry.append(cr)
            c +=1
    def run(self, circledic = None, time_int=0.01):
        t = time_int
        while True:
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
            for name in circledic:
                shape = circledic[name]
                if self.timestamp == name:
                    continue
                if self.circle.crosses(shape) or self.circle.touches(shape) or self.circle.intersects(shape):
                    self.ball_collision(shape)

            return self.circle
            
    def ball_collision(self,shape):
        [(x1,y1)],[(x2,y2)] = self.pnt.coords, shape.centroid.coords
        print('Ball collision!',[round(f,2) for f in [x1,y1,x2,y2]])
        angle = np.rad2deg(np.arctan2(y2 - y1, x2 - x1))
        d = shape.distance(self.pnt)
        self.p0 = [self.p0[0] - np.cos(np.deg2rad(angle))*d,self.p0[1] - np.sin(np.deg2rad(angle))*d]
        self.circle = self.pnt.buffer(self.r)
        
##        angle2 = np.rad2deg(np.arctan2(self.v0[1], self.v0[0]))
##        theta = angle + angle2
##        print('angle:',angle)
##        
##        v = (self.v0[0]**2+self.v0[1]**2)**0.5
##        vbi, vbj = v*np.sin(np.deg2rad(theta)), v*np.cos(np.deg2rad(theta))
##        vbj = -vbi *self.cor
##        v = (vbi**2+vbj**2)**0.5
##        angle2 = np.rad2deg(np.arctan2(vbj, vbi))
##        angle1 =angle-angle2
##        
##        self.v0 = [np.sin(np.deg2rad(angle1))*v, -np.cos(np.deg2rad(angle1))*v]
        
    def wall_collision(self,shape,p1,p2):
        
        pi,pf = list(shape.coords)
        angle = np.rad2deg(np.arctan2(pf[-1] - pi[-1], pf[0] - pi[0]))
        angle2 = np.rad2deg(np.arctan2(self.v0[1], self.v0[0]))
        v = (self.v0[0]**2+self.v0[1]**2)**0.5
        print('Wall collision')
        theta = angle - angle2
        vbi, vbj = v*np.sin(np.deg2rad(theta)), v*np.cos(np.deg2rad(theta))
        vbj = -vbj *self.cor
        v = (vbi**2+vbj**2)**0.5
        angle2 = np.rad2deg(np.arctan2(vbj, vbi))
        angle1 =angle+angle2
        
        self.v0 = [-np.sin(np.deg2rad(angle1))*v, np.cos(np.deg2rad(angle1))*v]
        #input('enter')
        
if __name__ == '__main__':
    plt.ion()
    fig,ax = plt.subplots(1,1)
    ax.autoscale()
    ax.relim()
    balldic = {}
    linedic = {}
    for i in range(50):
        app = BouncingBall()
        ball = app.timestamp
        balldic[ball] = app
        linedic[ball], = ax.plot([],[],'b')
        time.sleep(0.001)
    x,y = geo.box(-30,0,30,50).exterior.xy
    plt.plot(x,y)
    circledic = {}
    while plt.fignum_exists(fig.number):
        for ball in balldic:
            circle = balldic[ball].run(circledic)
            circledic[ball] = circle
        for ball in balldic:
            x,y = circledic[ball].exterior.xy
            linedic[ball].set_data(x,y)
        try:
            plt.draw()
            plt.pause(0.001)
        except:
            break
    print('Goodbye')
    time.sleep(.25)
    
