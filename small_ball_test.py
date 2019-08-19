import os, time, datetime, threading, random, sys, string
import numpy as np
from cortix.src.module import Module
from cortix.src.network import Network
from cortix.src.cortix_main import Cortix
import shapely.geometry as geo
import shapely.ops
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pandas as pd

class BallHandler(Module):

    def __init__(self,shape=None, balls= 5, runtime=3,color='b'):
        super().__init__()
        self.color = color
        self.shape = shape
        self.balls = balls
        self.runtime = runtime
        self.local_balls = []
        self.local_messengers = []
        self.collisions = 0
        self.r = 1
        self.t_step = 0.01
        self.timestamp=str(datetime.datetime.now())
        self.elapsed, self.oe = 0,0
        self.name = ''.join([random.choice(string.ascii_lowercase+string.digits) for f in range(10)])
        
    def run(self):
        t = 0.001
        for i in range(self.balls):
            ball = BouncingBall(shape=self.shape,color=self.color,r=self.r)
            ball.t_step = self.t_step
            self.local_balls.append(ball)
            self.local_messengers.append(ball.messenger)
        
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
            if round(self.elapsed) != self.oe:
                print('Elapsed Time:', round(self.elapsed))
                self.oe=round(self.elapsed)
            for ball in self.local_balls:
                collision = ball.run(ball_list)
                self.collisions+=collision
                
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
                self.send('Done',i)
        print('Done')
        return

class Plot(Module):
    def __init__(self, shape,modules=5,runtime=None):
        super().__init__()
        self.filetime = str(datetime.datetime.now())[:10]
        self.dir = '/tmp/bb'
        self.fps = 60
        if not os.path.exists(self.dir):
            os.makedirs(self.dir)
        self.filename = os.path.join(self.dir,'bb_data'+self.filetime+'.csv')
        self.length = modules
        self.shape = shape.buffer(0.5)
        self.timestamp=str(datetime.datetime.now())
        self.bndry = []
        coords = list(self.shape.exterior.coords)
        #Parse the box(LineRing) to create a list of line obstacles
        self.colordic = dict()
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
                lis = self.recv(i,timeout=4)
                if isinstance(lis,str):
                    c+=1
                    self.color=lis
                    print(c)
                    if c >=self.length:
                        self.plot()
                        return
                    continue
                
                for line in lis:
                    if line.name not in self.dic:
                        self.dic[line.name]=[]
                    self.dic[line.name].append(line.p)
                    self.colordic[line.name] = [line.color, line.r]
    def update(self,i):
        linelist = []
        for line in self.dic:
            pnt = geo.point.Point(self.dic[line][i])
            circle = pnt.buffer(self.colordic[line][1])
            x,y = circle.exterior.xy
            self.linedic[line].set_data(x,y)
            linelist.append(self.linedic[line])
        return linelist
    
    def init(self,ax):
##        linelist = []
##        for line in self.dic:
##            linelist.append(self.linedic[line])
        x,y = self.shape.exterior.xy
        ax.plot(x,y,'black')
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
                self.linedic[line], = ax.plot([],[],self.colordic[line][0])
        print('creating animation')
        ani = animation.FuncAnimation(fig, self.update, frames=[f for f in range(len(self.dic[line]))],
                            init_func=lambda:self.init(ax))
        plt.savefig('output_ball.png')
        ani.save('bb_animation.mp4', fps=self.fps)
        print('goodbye')
        return

class BouncingBall:
    def __init__(self,shape,bn=None,color='b',r=1):
        super().__init__()
        self.shape = shape

        self.bndry = []
        coords = list(self.shape.exterior.coords)
        self.bndic = dict()
        self.t_step = 0.01
        #Parse the box(LineRing) to create a list of line obstacles
        for c,f in enumerate(coords):
            try:
                cr = geo.LineString([coords[c],coords[c+1]])
                self.bndic[str(cr)] = geo.LineString([coords[c],coords[c+1]])
            except IndexError:
                cr = geo.LineString([coords[c],coords[-1]])
                self.bndic[str(cr)] = geo.LineString([coords[c],coords[-1]])
                           
                break
            
            self.bndry.append(cr)
        if bn==None:
            bn = self.shape.bounds
        self.r=r
        for i in range(100): #Attempt to spawn ball within boundary
            self.p0 = [random.uniform(bn[0],bn[2]),random.uniform(bn[1],bn[3])]
            self.pnt = geo.point.Point(self.p0[0],self.p0[1])
            self.circle = self.pnt.buffer(self.r)
            if self.shape.contains(self.circle):
                break
        if i>85:
            print('Warning, ball took:',i,'attempts to spawn')
        self.v0 = [random.uniform(-50,50),random.uniform(-30,30)]
        self.cor = 1.0
        self.a = (0,0)
        self.m = 1
        self.KE = 0.5*self.m*((self.v0[0]**2+self.v0[1]**2)**0.5)**2
        self.timestamp=str(datetime.datetime.now())
        self.name = ''.join([random.choice(string.ascii_lowercase+string.digits) for f in range(10)])
        #Customize container class that is sent to other modules
        self.messenger = Messenger()
        self.messenger.timestamp = self.timestamp
        self.messenger.m,self.messenger.r = self.m,self.r
        self.messenger.v = self.v0
        self.messenger.p = self.p0
        self.messenger.name = self.name
        self.messenger.color = color
    def run(self, ball_list):
        self.ball_list = ball_list
        self.collisions=0
        t = self.t_step

        #Gravity calculations for timestep
        self.p0[1] = 0.5*self.a[1]*t**2+self.v0[1]*t+self.p0[1]
        self.p0[0] = 0.5*self.a[0]*t**2+self.v0[0]*t+self.p0[0]
        self.v0[1] = self.a[1]*t + self.v0[1]
        self.v0[0] = self.a[0]*t + self.v0[0]
        
        #Update position and velocity variables
        self.pnt = geo.point.Point(self.p0[0],self.p0[1])
        self.circle = self.pnt.buffer(self.r)
        self.messenger.v = self.v0
        
        for shape in self.bndry: #Detects collision with boundary
            if self.circle.crosses(shape) or self.circle.touches(shape) or self.circle.intersects(shape):
                self.wall_collision(shape)
        for ball in self.ball_list: #Detects collision with other objects
            #ball is Messenger class object
            pnt = geo.point.Point(ball.p)
            shape = pnt.buffer(ball .r)
            name = ball.name
            if self.name==ball.name:
                continue
            for c,line in enumerate(ball.collision): #Undetected Collisions received as a message
                if self.name == line['name']:
                    p0,v0 = line['p0'],line['v0']
                    self.ball_collision(ball,p0,v0)
                    if self.circle.crosses(shape) or self.circle.touches(shape) or self.circle.intersects(shape):
                        self.ball_shift(shape)
                    del ball.collision[c]
            #Reacts to intersection between this object and another
            if self.circle.crosses(shape) or self.circle.touches(shape) or self.circle.intersects(shape):
                coldic = dict(name=self.name,v0=self.v0,p0=self.p0)
                self.messenger.collision.append(coldic)
                self.ball_collision(ball,ball.p,ball.v)
                self.ball_shift(shape)

        self.messenger.p = self.p0

        return self.collisions

    def wall_collision(self,shape):
        
        p1,p2 = shapely.ops.nearest_points(self.pnt,shape)
        angle3 = np.arctan2(p2.y - p1.y, p2.x - p1.x)
        d = shape.distance(self.pnt)
        self.p0 = [self.p0[0]-(self.r-d)*np.cos(angle3), self.p0[1]-(self.r-d)*np.sin(angle3)]
        self.pnt = geo.point.Point(self.p0[0],self.p0[1])
        self.circle = self.pnt.buffer(self.r)
        
        angle2 = np.arctan2(self.v0[1], self.v0[0])
        v = (self.v0[0]**2+self.v0[1]**2)**0.5
        theta = angle2-angle3
        vbi, vbj = v*np.sin(theta), v*np.cos(theta)
        vbj = -vbj *self.cor
        v = (vbi**2+vbj**2)**0.5
        angle4 = np.arctan2(vbj, vbi)
        angle1 = angle4 - angle3
        self.collisions+=1
        self.v0 = [np.sin(angle1)*v, np.cos(angle1)*v]

        
    def ball_shift(self,shape):
        p1,p2 = shapely.ops.nearest_points(self.pnt,shape)
        angle = np.arctan2(p2.y - p1.y, p2.x - p1.x)
        d = shape.distance(self.pnt)
        self.p0 = [self.p0[0]-(self.r*1.01-d)*np.cos(angle),self.p0[1]-(self.r*1.01-d)*np.sin(angle)]
        self.pnt = geo.point.Point(self.p0[0],self.p0[1])
        self.circle = self.pnt.buffer(self.r)
        
    def ball_collision(self,messenger,p0,v0):
##        pnt = geo.point.Point(p0[0],p0[1])
##        shape = pnt.buffer(messenger.r)
        v2,m = v0,messenger.m
        v3 = (v2[0]**2+v2[1]**2)**0.5
        phi = np.arctan2(v2[1],v2[0])
        p2x,p2y = p0[0],p0[1]
        angle = np.arctan2(p2y - self.p0[1], p2x - self.p0[0]) 
        angle2 = np.arctan2(self.v0[1], self.v0[0])
        v = (self.v0[0]**2+self.v0[1]**2)**0.5
        
        #Equation source: https://en.wikipedia.org/wiki/Elastic_collision
        vpx=((v*np.cos(angle2-angle)*(self.m-m)+2*m*v3*np.cos(phi-angle))/(self.m+m))*np.cos(angle)+v*np.sin(angle2-angle)*np.cos(angle+np.pi/2)
        vpy=((v*np.cos(angle2-angle)*(self.m-m)+2*m*v3*np.cos(phi-angle))/(self.m+m))*np.sin(angle)+v*np.sin(angle2-angle)*np.sin(angle+np.pi/2)
        vp = (vpx**2+vpy**2)**0.5
        self.v0 = [vpx,vpy]

class Messenger:
    def __init__(self, circle=None, collision = [], timestamp='0'):
        self.collision = [dict(name='',v0=[0.0,0.0],p0=[0.0,0.0])] #format
        self.collision = []
        self.timestamp = timestamp
        self.m = 1
        self.r = 1
        self.v = [0.0,0.0]
        self.p = [0.0,0.0]
        self.name = ''
        self.color = 'b'
        
class Simulation:
    def __init__(self):
        self.n_list = [15,]

        self.procs = 15
        self.runtime=30
        self.t_step = 0.01
        
        self.r=1
        self.mod_list = []
        self.shape = geo.Polygon([(0, 0), (0, 30), (30, 30),(30,0)])

        self.fps = 60

    def run(self):
        for c,i in enumerate(self.n_list):
            self.cortix = Cortix(use_mpi=False,log_filename = 'testing')
            self.net = Network()
            self.cortix.network = self.net
            self.plot = Plot(self.shape,modules=self.procs,runtime=self.runtime)
            self.plot.fps = self.fps
            self.net.add_module(self.plot)
            print(c,'iterations')
            self.balls = i
            self.balleach = int(self.balls/self.procs)
            self.mod_list = []
            for i in range(self.procs):    
                app = BallHandler(self.shape, balls=self.balleach,runtime = self.runtime)
                app.r=self.r
                app.t_step = 0.01
                self.mod_list.append(app)
                self.net.add_module(app)
            for c,i in enumerate(self.mod_list):
                self.net.connect([i,'plot-send{}'.format(c)],[self.plot,'plot-receive{}'.format(c)])
                for j in self.mod_list:
                    if i == j:
                        continue
                    name = '{}{}'.format(i.name,j.name)
                    name2 = '{}{}'.format(j.name,i.name)
                    self.net.connect([i,name], [j,name2])
            self.cortix.run()
            del self.cortix
            print('finished sim')

if __name__ == '__main__':
    sim = Simulation()
    sim.runtime = 0.1
    sim.r = 1
    sim.fps = 30
    sim.t_step = 0.01
    sim.procs = 3
    sim.n_list = [9]
    sim.run()