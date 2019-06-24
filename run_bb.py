import os, datetime, threading
from bouncingball import BouncingBall
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.backend_bases import key_press_handler
import tkinter as tk

class App:
    def __init__(self, parent):
        self.parent = parent
        self.ball_dic = dict()
        self.frame1=tk.Frame(master=parent)
        self.frame1.grid(column=0,row=0)
        self.frame2=tk.Frame(master=parent)
        self.frame2.grid(column=0,row=1)
        self.frame3=tk.Frame(master=parent)
        self.frame3.grid(column=0,row=2)

        self.fig,self.ax = plt.subplots(1,1)
        self.ax.set_ylim(0,50)
        self.ax.set_xlim(-30,30)
        self.ax.set_title('Boucing Balls!')
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.frame1)
        self.canvas.get_tk_widget().pack()

        self.toolbar = NavigationToolbar2Tk(self.canvas, self.frame1)
        self.toolbar.update()
        self.canvas._tkcanvas.pack()
        self.canvas.mpl_connect("key_press_event", self.on_key_press)
        self.canvas.draw()

        lbutton = tk.Button(master=self.frame2, text="Ball", command=self.add_ball)
        lbutton.grid(column=0,row=0)
        
        tk.Button(master=self.frame2, text="Clear", command=self.clear_balls).grid(column=1,row=0)
        self.parent.after(0, self.update_plot)
    def clear_balls(self):
        self.ball_dic=dict()
        del self.ax.lines[:1000000]
        self.canvas.draw()
    def update_plot(self):
        for ball in self.ball_dic:
            x,y = ball.run(time_int=0.1)
            self.ball_dic[ball].set_data(x,y)
        self.ax.set_title('Boucing Balls! {} balls'.format(len(self.ball_dic)))
        self.canvas.draw()
        self.parent.after(12, self.update_plot)
    def add_ball(self):
        ball = BouncingBall()
        self.ball_dic[ball], = self.ax.plot([],[],'ob')
    def on_key_press(self, event):
        key_press_handler(event, self.canvas, self.toolbar)
        self.toolbar.update()
if __name__ == '__main__':
    root = tk.Tk()
    app = App(root)
    root.mainloop()
