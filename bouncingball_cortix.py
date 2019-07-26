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

#global sends, receives
sends = 0
receives = 0
class BouncingBall(Module):
    def __init__(self):
        super().__init__()
        self.cor = 0.95
        self.p0 = [random.randint(0,10),random.randint(10,30)]
        r=1.0
        self.ucircle = [[],[]]
        self.circle=[[],[]]
        for phi in np.arange(0,6.28,0.01):
            self.ucircle[0].append(r*np.cos(phi))
            self.ucircle[1].append(r*np.sin(phi))
        self.circle[0] = [f+self.p0[0] for f in self.ucircle[0]]
        self.circle[1] = [f+self.p0[1] for f in self.ucircle[1]]
        self.v0 = [random.uniform(-70,70),random.uniform(-40,40)]
        self.cor = 0.95
        self.a = (0,-9.81)
        self.timestamp=str(datetime.datetime.now())
        
    def run(self, time_int=0.0005):
        t = time_int
        for i in range(10000):
            self.py = 0.5*self.a[1]*t**2+self.v0[1]*t+self.p0[1]
            self.px = 0.5*self.a[0]*t**2+self.v0[0]*t+self.p0[0]
            self.vy = self.a[1]*t + self.v0[1]
            self.vx = self.a[0]*t + self.v0[0]
            self.v0[0],self.v0[1]=self.vx,self.vy
            self.p0[0],self.p0[1]=self.px,self.py
            self.circle[0] = [f+self.p0[0] for f in self.ucircle[0]]
            self.circle[1] = [f+self.p0[1] for f in self.ucircle[1]]
            if min(self.circle[1]) <0:
                self.v0[1] = -self.v0[1]*self.cor
                self.p0[0],self.p0[1] = self.px,0.01+1
                self.circle[0] = [f+self.p0[0] for f in self.ucircle[0]]
                self.circle[1] = [f+self.p0[1] for f in self.ucircle[1]]
                t=0.0
            if max(self.circle[1]) > 50:
                self.v0[1] = -self.v0[1]*self.cor
                self.p0[0],self.p0[1] = self.px,49.99-1
                self.circle[0] = [f+self.p0[0] for f in self.ucircle[0]]
                self.circle[1] = [f+self.p0[1] for f in self.ucircle[1]]
                t=0.0
            if min(self.circle[0]) <-30.0:
                self.v0[0] = -self.v0[0]*self.cor
                self.p0[0], self.p0[1] = -29.99+1, self.py
                self.circle[0] = [f+self.p0[0] for f in self.ucircle[0]]
                self.circle[1] = [f+self.p0[1] for f in self.ucircle[1]]
                t=0.0
            if max(self.circle[-0]) > 30:
                self.v0[0] = -self.v0[0]*self.cor
                self.p0[0], self.p0[1] = 29.99-1,self.py
                self.circle[0] = [f+self.p0[0] for f in self.ucircle[0]]
                self.circle[1] = [f+self.p0[1] for f in self.ucircle[1]]
                t=0
            for i in self.ports:
                self.send(self.circle,i)
            for i in self.ports:
                if 'plot' in str(i):
                    continue
                self.circle = self.recv(i)
            print(self.v0) 
        for i in self.ports:
            if 'plot' in str(i):
                self.send('done',i)
        print('done')
        return
    def wall_collision(self):
        pass


if __name__ == '__main__':
    cortix = Cortix(use_mpi=False)
    mod_list = []
    plot = Plot()
    cortix.add_module(plot)
    for i in range(2):
        time.sleep(0.01)
        app = BouncingBall()
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
    
            

    print('Goodbye')
    time.sleep(5)
