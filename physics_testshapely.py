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
        self.r=3.0
        self.ucircle = [[],[]]
        self.circle=[[],[]]
##        for phi in np.arange(0,6.28,0.02):
##            self.ucircle[0].append(r*np.cos(phi))
##            self.ucircle[1].append(r*np.sin(phi))
        self.circle[0] = [f+self.p0[0] for f in self.ucircle[0]]
        self.circle[1] = [f+self.p0[1] for f in self.ucircle[1]]
        self.v0 = [random.uniform(0,70),random.uniform(-5,5)]
        self.p0o = self.p0
        self.v0o = self.v0
        self.cor = 0.95
        self.a = (0,-9.81)
        self.timestamp=str(datetime.datetime.now())
        self.box = geo.box(-30,0,30,50)
        self.box = affinity.rotate(self.box,40)
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
    def run(self, time_int=0.09):
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

                        
##            for b in self.bndry:
##                for c,(p,q) in enumerate(zip(self.circle[0],self.circle[1])):
##                    pass        
##                if q < 0:
##                    self.circle[1][c] = 0
##                    self.v0[1] = abs(self.v0[1])
##                    t=0.0
##                if p <-30:
##                    self.circle[0][c] = -30
##                    self.v0[0] = abs(self.v0[0])
##                    t=0.0
##                if p > 30:
##                    self.circle[0][c] = 30
##                    self.v0[0] = -abs(self.v0[0])
##                    t=0.0
            x,y = self.circle.exterior.xy
            self.p0o = self.p0
            self.v0o = self.v0
            return x,y
            
            
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
        #input('enter')
##circle = geo.point.Point(1,1).buffer(1)
##x,y = circle.exterior.xy

##box = geo.box(-30,0,30,50)
##bndry=[]
##c=0
##coords = list(box.exterior.coords)
##for f in coords:
##    try:
##        cr = geo.LineString([coords[c],coords[c+1]])
##    except IndexError:
##        cr = geo.LineString([coords[c],coords[-1]])
##        break
##    
##    bndry.append(cr)
##    c +=1
##for f in bndry:
##    x,y = f.xy
##    plt.plot(x,y)
        
plt.show()
if __name__ == '__main__':
    plt.ion()
    fig,ax = plt.subplots(1,1)
    line, = ax.plot([],[],'b')
    line2, = ax.plot([],[],'b')
    line3, = ax.plot([],[],'b')
    ax.autoscale()
    ax.relim
    app = BouncingBall()
    app2 = BouncingBall()
    app3 = BouncingBall()
    x,y = affinity.rotate(geo.box(-30,0,30,50),45).exterior.xy
    plt.plot(x,y)
    while plt.fignum_exists(fig.number):
        x,y = app.run()
        line.set_data(x,y)
        x,y = app2.run()
        line2.set_data(x,y)
        x,y = app3.run()
        line3.set_data(x,y)
        try:
            plt.draw()
            plt.pause(0.001)
        except:
            break
    print('Goodbye')
    time.sleep(.25)
    
