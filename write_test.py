import os, time, datetime, threading, random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from cortix.src.module import Module
from cortix.src.port import Port
from cortix.src.cortix_main import Cortix
import shapely.geometry as geo
import shapely.ops

class Writer(Module):
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
                        return
                    continue
                circle = messenger.circle
                x,y = circle.exterior.xy
                with open('trajectories.csv','a') as f:
                    f.write('{},{}\n'.format(x,y))
            
