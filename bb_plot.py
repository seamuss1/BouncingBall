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
    def __init__(self,shape = None, length=5):
        super().__init__()
        self.length = length
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

    def run(self, state_comm=None, idx_comm=None):
        state_comm.put((idx_comm,self.state))
        print('start plot')
        self.dic = {}
        c = 0
        while True:
            for i in self.ports:
                if not 'plot' in str(i):
                    continue
                if str(i) not in self.dic:
                    self.dic[str(i)] = []
                messenger = self.recv(i) 
                if str(messenger) == 'done':
                    c+=1
                    if c >=self.length:
                        self.plot()
                        return
                    continue
                circle = messenger.circle
                x,y = circle.exterior.xy
                self.dic[str(i)].append([x,y])
            


    def update(self,i):
        linelist = []
        for line in self.dic:    
            x,y = self.dic[line][i][0],self.dic[line][i][1]
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
        ani.save('bb_animation.mp4', fps=60)
        print('goodbye')
        return
