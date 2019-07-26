import os, time, datetime, threading, random
import numpy as np
import matplotlib.pyplot as plt
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
        while True:
            for i in self.ports:
                if not 'plot' in str(i):
                    continue
                if str(i) not in self.dic:
                    self.dic[str(i)] = []
                circle = self.recv(i)
               
            
                if str(circle) == 'done':
                    self.plot()
                    return
                self.dic[str(i)].append(circle)
    def plot(self):
        print('starting plot')
        for line in self.dic:
            #fig,ax = plt.subplots(111)
            for i in self.dic[line]:
                plt.plot(i[0],i[1])
        plt.savefig('output_ball.png')
        print('goodbye')
        return
