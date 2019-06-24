import os, time, datetime, threading, random
import numpy as np
import matplotlib.pyplot as plt
from cortix.src.module import Module
import time as Module


class BouncingBall:#(Module):
    def __init__(self):
        super().__init__()
        self.cor = 0.95
        self.p0 = [random.randint(0,10),random.randint(10,30)]
        r=1.0
        self.circle = ([],[])
        for phi in np.arange(0,6.28,0.01):
            self.circle[0].append(r*np.cos(phi))
            self.circle[1].append(r*np.sin(phi))
        self.v0 = [random.uniform(-70,70),random.uniform(-40,40)]
        self.cor = 0.95
        self.a = (0,-9.81)
        self.timestamp=str(datetime.datetime.now())
    def run(self, time_int=0.1):
        t = time_int
        while True:
            self.py = 0.5*self.a[1]*t**2+self.v0[1]*t+self.p0[1]
            self.px = 0.5*self.a[0]*t**2+self.v0[0]*t+self.p0[0]
            self.vy = self.a[1]*t + self.v0[1]
            self.vx = self.a[0]*t + self.v0[0]
            self.v0[0],self.v0[1]=self.vx,self.vy
            self.p0[0],self.p0[1]=self.px,self.py
            if self.py <0:
                self.v0[1] = -self.v0[1]*self.cor
                self.p0[0],self.p0[1] = self.px,0.01
                t=0.0
            if self.px<-30.0:
                self.v0[0] = -self.v0[0]*self.cor
                self.p0[0], self.p0[1] = -29.99, self.py
                t=0.0
            if self.px > 30:
                self.v0[0] = -self.v0[0]*self.cor
                self.p0[0], self.p0[1] = 29.99,self.py
                t=0.0
            return self.px,self.py
##            self.send((px,py), 'bb-plot')
            
            

##        self.send('DONE', 'bb-plot')
    def wall_collision(self):
        
        pass





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
            print('Goodbye')
            time.sleep(5)
            break
