import os, time, datetime, threading, random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from cortix.src.module import Module
from cortix.src.port import Port
from cortix.src.cortix_main import Cortix
import shapely.geometry as geo
import shapely.ops

class Plot(Module):
    def __init__(self,shape = None, modules=5):
        super().__init__()
        self.filetime = str(datetime.datetime.now())[:10]
        self.dir = '/tmp/bb'
        self.length = 0
        self.fps = 60
        if not os.path.exists(self.dir):
            os.makedirs(self.dir)
        self.filename = os.path.join(self.dir,'bb_data'+self.filetime+'.csv')
        self.length = modules
        self.shape = shape
        self.timestamp=str(datetime.datetime.now())
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

    def run(self):
        print('start plot')
        self.dic = {}
        c = 0
        while True:
            for i in self.ports:
                if not 'plot' in str(i):
                    continue
                lis = self.recv(i)
                if isinstance(lis,str):
                    c+=1
                    if c >=self.length:
                        self.plot()
                        return
                    continue
                
                for line in lis:
                    if line.name not in self.dic:
                        self.dic[line.name]=[]
                    self.dic[line.name].append(line.circle)
    def update(self,i):
        linelist = []
        for line in self.dic:
            x,y = self.dic[line][i].exterior.xy
            self.linedic[line].set_data(x,y)
            linelist.append(self.linedic[line])
        return linelist
    
    def init(self,ax):
        for f in self.bndry:
            x,y = f.xy
            ax.plot(x,y,'r')
        
        ax.autoscale()
        ax.set_aspect( 'equal', adjustable='datalim')
        ax.relim()
        return ax
    def plot(self):
        print('starting plot')
        self.linedic = dict()
        fig = plt.figure()
        ax = fig.add_subplot(111)
        for line in self.dic:
            if line not in self.linedic:
                self.linedic[line], = ax.plot([],[],'b')
        print('creating animation')
        ani = animation.FuncAnimation(fig, self.update, frames=[f for f in range(len(self.dic[line]))],
                            init_func=lambda:self.init(ax), blit=False)
        plt.savefig('output_ball.png')
        ani.save('bb_animation.mp4', fps=self.fps)
        print('goodbye')
        return
