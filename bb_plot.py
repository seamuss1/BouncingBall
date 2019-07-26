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
        print('plot')
        return
##        for i in self.ports:
##            if not 'plot' in str(i):
##                print('check')
##                continue
##            circle = self.recv(i)
##            print(circle)
##            
##            if str(circle) == 'done':
##                return
##        return
