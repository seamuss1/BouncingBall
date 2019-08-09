import os, time, datetime, threading, random
import numpy as np
import shapely.geometry as geo
import shapely.ops
from shapely import affinity

class BouncingBall:
    def __init__(self,shape):
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
        bn = self.shape.bounds
        self.r=1.0
        for i in range(100): #Attempt to spawn ball within boundary
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
        time.sleep(0.05)
        self.name = str(datetime.datetime.now())
        
        #Customize container class that is sent to other modules
        self.messenger = Messenger()
        self.messenger.circle = self.circle
        self.messenger.timestamp = self.timestamp
        self.messenger.m,self.messenger.r = 1,1
        self.messenger.v = self.v0
        self.messenger.p = self.p0
        self.messenger.name = self.name
        
    def run(self, ball_list):
        self.ball_list = ball_list
        t = 0.01

        
        #Gravity calculations for timestep
        self.p0[1] = 0.5*self.a[1]*t**2+self.v0[1]*t+self.p0[1]
        self.p0[0] = 0.5*self.a[0]*t**2+self.v0[0]*t+self.p0[0]
        self.v0[1] = self.a[1]*t + self.v0[1]
        self.v0[0] = self.a[0]*t + self.v0[0]
        
        #Update position and velocity variables
        self.pnt = geo.point.Point(self.p0[0],self.p0[1])
        self.circle = self.pnt.buffer(self.r)
        self.messenger.v = self.v0
        
        for shape in self.bndry: #Detects collision with boundary
            if self.circle.crosses(shape) or self.circle.touches(shape) or self.circle.intersects(shape):
                self.wall_collision(shape)
        for ball in self.ball_list: #Detects collision with other objects
            #ball is Messenger class object
            shape = ball.circle
            name = ball.name
            if self.name==ball.name:
                continue
            for line in ball.collision: #Undetected Collisions received as a message
                if self.name == line:
                    self.ball_collision(ball)
                    if self.circle.crosses(shape) or self.circle.touches(shape) or self.circle.intersects(shape):
                        self.ball_shift(shape)
            #Reacts to intersection between this object and another
            if self.circle.crosses(shape) or self.circle.touches(shape) or self.circle.intersects(shape):
                self.ball_collision(ball)
                self.ball_shift(shape)
                self.messenger.collision.append(name)

        self.messenger.circle = self.circle
        self.messenger.p = self.p0
        
        self.messenger.collision = [] #Reset list of collisions

        return self.messenger

    def wall_collision(self,shape):
        p1,p2 = shapely.ops.nearest_points(self.pnt,shape)
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
    def ball_shift(self,shape):
        p1,p2 = shapely.ops.nearest_points(self.pnt,shape)
        angle = np.arctan2(p2.y - p1.y, p2.x - p1.x)
        d = shape.distance(self.pnt)
        self.p0 = [self.p0[0]-(self.r*1.01-d)*np.cos(angle),self.p0[1]-(self.r*1.01-d)*np.sin(angle)]
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
        vpx=((v*np.cos(angle2-angle)*(self.m-m)+2*m*v3*np.cos(phi-angle))/(self.m+m))*np.cos(angle)+v*np.sin(angle2-angle)*np.cos(angle+np.pi/2)
        vpy=((v*np.cos(angle2-angle)*(self.m-m)+2*m*v3*np.cos(phi-angle))/(self.m+m))*np.sin(angle)+v*np.sin(angle2-angle)*np.sin(angle+np.pi/2)
        vp = (vpx**2+vpy**2)**0.5
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
        self.name = ''
if __name__ == '__main__':
    print('Goodbye')
    time.sleep(5)
