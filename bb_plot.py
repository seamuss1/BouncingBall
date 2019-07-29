import os, time, datetime, threading, random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from cortix.src.module import Module
from cortix.src.port import Port
from cortix.src.cortix_main import Cortix


class Plot(Module):
    def __init__(self):
        super().__init__()
        self.timestamp=str(datetime.datetime.now())
    def run(self):
        print('start plot')
        self.dic = {}
        c = 0
        while True:
            for i in self.ports:
                if not 'plot' in str(i):
                    continue
                if str(i) not in self.dic:
                    self.dic[str(i)] = []
                circle = self.recv(i)
                if str(circle) == 'done':
                    c+=1
                    if c >=3:
                        self.plot()
                        return
                    continue
                self.dic[str(i)].append(circle)
            


    def update(self,i):
        linelist = []
        for line in self.dic:    
            x,y = self.dic[line][i][0],self.dic[line][i][1]
            self.linedic[line].set_data(x,y)
            linelist.append(self.linedic[line])
        return linelist
    def init(self,ax):
        ax.set_xlim(-30,30)
        ax.set_ylim(0,50)
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
        ani.save('bb_animation.mp4', fps=30)
        print('goodbye')
        return
