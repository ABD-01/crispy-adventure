import time
from tkinter import *
import numpy as np

from enum import Enum

class State(Enum):
    forward = 0
    rotating = 1

simHeight = 405
simWidth = 405

class Circle:

    def __init__(self, coords, diameter, velocity=None):
        self.coords = coords
        self.radius = diameter/2
        if velocity is None:
            self.velocity = self.get_random_velocity()
        else:
            self.velocity = velocity
        self.state = State.forward
        theta = np.deg2rad(3)
        self.rotationMat = np.array([[np.cos(theta), np.sin(theta)],
                                     [-np.sin(theta), np.cos(theta)]])
        self.rotateStartTime = 0
        self.rotateTime = 0
    
    def checkCollision(self, boundary):
        return (
            np.any(self.coords - np.array([self.radius, self.radius]) < 0)
            or
            np.any(self.coords + np.array([self.radius, self.radius]) > boundary)
        )

    def move(self, dt, maxX, maxY):
        if self.state == State.forward:
            if self.checkCollision(np.array([maxX, maxY])):
                self.velocity = self.rotationMat.dot(self.velocity)
                self.state = State.rotating
                self.rotateStartTime = time.time()
                self.rotateTime = np.random.uniform(0, 1)
                print("Rotate for " + str(self.rotateTime))
            else:
                self.coords += self.velocity*dt
        else:
            if time.time() - self.rotateStartTime > self.rotateTime:
                print("Done rotating")
                self.state = State.forward
                self.coords += self.velocity*dt
            else:
                print("Rotating. Time left: " + str(self.rotateTime - (time.time() - self.rotateStartTime)))
                self.velocity = self.rotationMat.dot(self.velocity)
                curr_coords = self.coords.copy()
                self.coords += self.velocity*dt
                if self.checkCollision(np.array([maxX, maxY])):
                    self.coords = curr_coords

    def get_random_velocity(self):
        minVel, maxVel = 100, 900
        mu, sigma = 500, 200
        
        velMag = np.random.normal(mu, sigma, size=(2,))
        velMag = np.clip(velMag, minVel, maxVel)
        
        vel = velMag * np.random.choice([-1, 1], size=(2,))
        
        return vel 

    def draw(self, canvas, **kwargs):
        x0,y0 = self.coords - self.radius
        x1,y1 = self.coords + self.radius
        self.circleDw = canvas.create_oval(x0,y0,x1,y1,  **kwargs)
        arrow_v = (self.velocity / np.linalg.norm(self.velocity)) * self.radius
        self.arrowDw = canvas.create_line(*self.coords, *(self.coords + arrow_v), fill=kwargs.get("linefill",  "red"), width=2, arrow=LAST)
    
    def clean(self, canvas):
        canvas.delete(self.circleDw)
        canvas.delete(self.arrowDw)

class Display:
    def __init__(self, maxX = simWidth, maxY = simHeight, timDelta=0.001):
        self.maxX = maxX
        self.maxY = maxY
        self.dt = timDelta
        self.win = Tk(className="Simulator")

        self.canvas = Canvas(self.win, width=simWidth, height=simHeight, background="white")

        self.canvas.create_rectangle(0,0,5, simHeight-5, fill='grey', outline='black', width=2)
        self.canvas.create_rectangle(0, simHeight-5 , simWidth-5, simHeight, fill='grey', outline='black', width=2)
        self.canvas.create_rectangle(simWidth-5, 5 , simWidth, simHeight, fill='grey', outline='black', width=2)
        self.canvas.create_rectangle(5,0,simWidth, 5, fill='grey', outline='black', width=2)
        self.canvas.pack()

        start_button = Button(self.win, text="Start", command=self.step)
        start_button.pack()

        self.circle = Circle(np.array([300.0,300.0]), 30)

        self.start()

        print("Strating Main loop")
        self.win.mainloop()

    def start(self):
        self.circle.draw(self.canvas, fill="black")
        self.canvas.update()
        self.win.after(10, self.step)

    def step(self):
        self.circle.clean(self.canvas)
        self.circle.move(self.dt, self.maxX-5, self.maxY-5)
        self.circle.draw(self.canvas, fill="black")
        self.canvas.update()
        self.win.after(int(self.dt*1000), self.step)

Display()
