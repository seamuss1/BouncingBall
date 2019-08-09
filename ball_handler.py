import os, time, datetime, threading, random, sys
import numpy as np
from cortix.src.module import Module
from cortix.src.port import Port
from cortix.src.cortix_main import Cortix
from plot_multi import Plot
from bouncingball import BouncingBall
import shapely.geometry as geo

class BallHandler(Module):
    def __init__(self,shape=None, balls= 5, runtime=3):
        super().__init__()
        self.shape = shape
        self.balls = balls
        self.runtime = runtime
        self.local_balls = []
        self.local_messengers = []
        self.timestamp=str(datetime.datetime.now())
        for i in range(self.balls):
            ball = BouncingBall(self.shape)
            self.local_balls.append(ball)
            self.local_messengers.append(ball.messenger)
    def run(self):
        t = 0.01
        self.elapsed, oe = 0,0
        its = round(self.runtime/t)
        ball_list = []
        
        for messenger in self.local_messengers:
            ball_list.append(messenger)
        for i in self.ports: #Send initial properties
            if 'plot' not in str(i):
                self.send(self.local_messengers,i)
        
        for i in self.ports:
            if 'plot' not in str(i):
                ex_balls = self.recv(i)
                for ball in ex_balls:
                    ball_list.append(ball)
        
        for i in range(its):
            self.elapsed += t
            if oe != round(self.elapsed,1):
                print('Time Elapsed: ', round(self.elapsed,1))
                oe = round(self.elapsed,1)
            for ball in self.local_balls:
                ball.run(ball_list)
                
            for i in self.ports: #Send and receive messages for each timestep
                self.send(self.local_messengers,i)
            ball_list = [f for f in self.local_messengers]   
            for i in self.ports:
                if 'plot' in str(i): #Not receiving messages from plotting
                    continue
                messengerlis = self.recv(i)
                for messenger in messengerlis:
                    ball_list.append(messenger)
                
            for ball in self.local_balls:
                ball.messenger.collision = []

        for i in self.ports: #Send 'done' string to plot module as end condition
            if 'plot' in str(i):
                self.send('done',i)
        print('Time Elapsed: ', self.elapsed)
        print('Done')
if __name__ == '__main__':
    cortix = Cortix(use_mpi=False)
    mod_list = []
    shape = geo.box(-30,0,30,50)
    
    plot = Plot(shape=shape, modules=10)
    cortix.add_module(plot)

    for i in range(10):
        time.sleep(0.01)
        app = BallHandler(shape, balls=10,runtime = 3)
        mod_list.append(app)
        cortix.add_module(app)
        
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
