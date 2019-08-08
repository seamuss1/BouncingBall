import os, time, datetime, threading, random
import numpy as np
import matplotlib.pyplot as plt
import sys
from cortix.src.module import Module
from cortix.src.port import Port
from cortix.src.cortix_main import Cortix
import time
from bb_plot import Plot
from write_test import Writer
from bouncingball_cortix import BouncingBall
import shapely.geometry as geo
import shapely.ops
from shapely import affinity

def test_ball():
    cortix = Cortix(use_mpi=False)
    secs = 4
    mod_list = []
    shape = geo.box(-30,0,30,50)
    plot = Plot(shape=shape, length=2)
    cortix.add_module(plot)
    ball1 = BouncingBall(shape,runtime=secs)
    ball1.p0 = [0,20]
    ball1.v0 = [20,4]
    mod_list.append(ball1)
    cortix.add_module(ball1)
    time.sleep(0.01)
    ball2 = BouncingBall(shape,runtime=secs)
    ball2.p0 = [17,22]
    ball2.v0 = [-5,1]
    mod_list.append(ball2)
    cortix.add_module(ball2)    
                
    for c,i in enumerate(mod_list):
        i.connect('plot-send{}'.format(c),plot.get_port('plot-receive{}'.format(c)))
        for j in mod_list:
            if i == j:
                continue
            name = '{}{}'.format(i.timestamp,j.timestamp)
            name2 = '{}{}'.format(j.timestamp,i.timestamp)
            j.connect(name, i.get_port(name2))
            
    cortix.draw_network('network_graph.png')
    cortix.run()
    print('bye')

if __name__ == '__main__':
    test_ball()    
