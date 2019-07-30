import os, time, datetime, threading, random
import numpy as np
import matplotlib.pyplot as plt
from cortix.src.module import Module
import time 
import shapely.geometry as geo
import shapely.ops

class BouncingBall:#(Module):
    def __init__(self):
        super().__init__()
        self.cor = 0.95
        self.p0 = [random.randint(0,10),random.randint(10,30)]
        self.r=1.0
        self.ucircle = [[],[]]
        self.circle=[[],[]]
##        for phi in np.arange(0,6.28,0.02):
##            self.ucircle[0].append(r*np.cos(phi))
##            self.ucircle[1].append(r*np.sin(phi))
        self.circle[0] = [f+self.p0[0] for f in self.ucircle[0]]
        self.circle[1] = [f+self.p0[1] for f in self.ucircle[1]]
        self.v0 = [random.uniform(0,30),random.uniform(-5,5)]
        self.p0o = self.p0
        self.v0o = self.v0
        self.cor = 0.95
        self.a = (0,-9.81)
        self.timestamp=str(datetime.datetime.now())
        self.bndry = []
        self.bndry.append([np.linspace(-30,-30),np.linspace(0,50)])
        self.bndry.append([np.linspace(30,30),np.linspace(0,50)])
        self.bndry.append([np.linspace(-30,30),np.linspace(50,50)])
        self.bndry.append([np.linspace(-30,30),np.linspace(0,0)])
        self.box = geo.box(-30,0,30,50)

    def run(self, time_int=0.01):
        t = time_int
        while True:
            self.p0[1] = 0.5*self.a[1]*t**2+self.v0[1]*t+self.p0[1]
            self.p0[0] = 0.5*self.a[0]*t**2+self.v0[0]*t+self.p0[0]
            self.v0[1] = self.a[1]*t + self.v0[1]
            self.v0[0] = self.a[0]*t + self.v0[0]
            self.pnt = geo.point.Point(self.p0[0],self.p0[1])
            self.circle = self.pnt.buffer(self.r)
            if not(self.box.contains(self.circle)):
                print('DOES NOT CONTAIN')
                for x in range(100):
                    i = t/100
                    self.p0o[1] = 0.5*self.a[1]*i**2+self.v0o[1]*i+self.p0o[1]
                    self.p0o[0] = 0.5*self.a[0]*i**2+self.v0o[0]*i+self.p0o[0]
                    self.v0o[1] = self.a[1]*t + self.v0o[1]
                    self.v0o[0] = self.a[0]*t + self.v0o[0]
                    self.v0,self.p0=self.v0o,self.p0o
                    self.pnt = geo.point.Point(self.p0[0],self.p0[1])
                    self.circle = self.pnt.buffer(self.r)
                    if not(self.box.contains(self.circle)):
                        self.wall_collision()
                        
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
            
            
    def wall_collision(self):
        
            
        print('distance',self.pnt.distance(self.box))
        p1,p2 = shapely.ops.nearest_points(self.box,self.pnt)
        x1,y1,x2,y2 = p1.x,p1.y, p2.x,p2.y
        v = np.sqrt(np.square(self.v0[0])+np.square(self.v0[1]))
        print(x1,y1,x2,y2)
        print(v)
        input('enter')
##circle = geo.point.Point(1,1).buffer(1)
##x,y = circle.exterior.xy

if __name__ == '__main__':
    plt.ion()
    fig,ax = plt.subplots(1,1)
    line, = ax.plot([],[],'bo')
    line2, = ax.plot([],[],'bo')
    line3, = ax.plot([],[],'bo')
    ax.set_ylim(0,50)
    ax.set_xlim(-30,30)
    app = BouncingBall()
    app2 = BouncingBall()
    app3 = BouncingBall()
    x,y = geo.box(-30,0,30,50).exterior.xy
    plt.plot(x,y)
    while plt.fignum_exists(fig.number):
        x,y = app.run(time_int=0.1)
        line.set_data(x,y)
        x,y = app2.run(time_int=0.1)
        line2.set_data(x,y)
        x,y = app3.run(time_int=0.1)
        line3.set_data(x,y)
        try:
            plt.draw()
            plt.pause(0.001)
        except:
            break
    print('Goodbye')
    time.sleep(.5)
    
