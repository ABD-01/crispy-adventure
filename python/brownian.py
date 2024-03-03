import time
from tkinter import *
import numpy as np

from enum import Enum
import argparse

class State(Enum):
    """
    :meta private:
    """
    forward = 0
    rotating = 1


class Circle:
    """Class representing a point/circle with Brownian motion.
    """

    def __init__(self, coords, diameter, velocity=None, theta=3, stop_and_rotate=False):
        """
        Parameters:
            coords (numpy.ndarray): The initial coordinates of the circle.
            diameter (int): Diameter of the circle.
            velocity (numpy.ndarray, optional): Initial velocity of the circle. If None, a random velocity is assigned.
            theta (int, optional): Angular velocity for rotation. (Defaul: ``3``)
            stop_and_rotate (bool, optional): This is because of the the ambigous requirement of the challenge (Default: ``False``)
        """
        self.coords = coords
        self.radius = diameter/2
        if velocity is None:
            self.velocity = self.get_random_velocity()
        else:
            self.velocity = velocity
        self.state = State.forward
        theta = np.deg2rad(theta)
        self.rotationMat = np.array([[np.cos(theta), np.sin(theta)],
                                     [-np.sin(theta), np.cos(theta)]])
        self.rotateStartTime = 0
        self.rotateTime = 0
        self.stop_and_rotate = stop_and_rotate
    
    def checkCollision(self, coords, boundary):
        """
        Check if the circle collides with the specified boundary.

        Parameters:
            coords (numpy.ndarray): Current coordinates of the circle.
            boundary (numpy.ndarray): Boundary dimensions.

        Returns:
            (bool): True if collision occurs, False otherwise.

        """
        return (
            np.any(coords - np.array([self.radius, self.radius]) < 0)
            or
            np.any(coords + np.array([self.radius, self.radius]) > boundary)
        )

    def move(self, dt, maxX, maxY):
        """Move the circle based on its state.
        
        Parameters:
            dt: Time step for movement.
            maxX: Maximum x-coordinate for the boundary.
            maxY: Maximum y-coordinate for the boundary.

        """
        if self.state == State.forward:
            if self.checkCollision(self.coords, np.array([maxX, maxY])):
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
                while self.checkCollision(self.coords+self.velocity*dt ,np.array([maxX, maxY])):
                    self.velocity = self.rotationMat.dot(self.velocity)
                self.coords += self.velocity*dt
            else:
                print("Rotating. Time left: " + str(self.rotateTime - (time.time() - self.rotateStartTime)))
                self.velocity = self.rotationMat.dot(self.velocity)
                if not self.stop_and_rotate:
                    if not self.checkCollision(self.coords+self.velocity*dt ,np.array([maxX, maxY])):
                        self.coords += self.velocity*dt

    def get_random_velocity(self):
        r"""Generate a random velocity vector.

        Samples velocity magnitude from a
        `normal distribution <https://en.wikipedia.org/wiki/Normal_distribution>`_
        :math:`\mathcal{N}(\mu, \sigma^2)`,
        where :math:`\mu = 500` and :math:`\sigma = 200`, and clips it as 

        .. math::
            \lVert V \rVert = \begin{cases}
                V, & \text{if} \ V_{\text{min}} \leq V \leq V_{\text{max}} \\
                V_{\text{min}}, & \text{if} \ V \leq V_{\text{min}} \\
                V_{\text{max}}, & \text{if} \ V \geq V_{\text{max}}
            \end{cases}

        where :math:`V_{\text{min}} = 100` and :math:`V_{\text{max}} = 900`.
        
        Then randomly assigns direction to the velocity.

        Returns:
            (numpy.ndarray): Random velocity vector :math:`V`.
        """
        minVel, maxVel = 100, 900
        mu, sigma = 500, 200
        
        velMag = np.random.normal(mu, sigma, size=(2,))
        velMag = np.clip(velMag, minVel, maxVel)
        
        vel = velMag * np.random.choice([-1, 1], size=(2,))
        
        return vel 

    def draw(self, canvas, **kwargs):
        """
        Draws the circle on the canvas.

        Parameters:
            canvas (`tk.Canvas <https://tkdocs.com/shipman/canvas.html>`_): Canvas object for drawing.
            **kwargs: Additional keyword arguments for drawing.

        """
        x0,y0 = self.coords - self.radius
        x1,y1 = self.coords + self.radius
        self.circleDw = canvas.create_oval(x0,y0,x1,y1,  **kwargs)
        arrow_v = (self.velocity / np.linalg.norm(self.velocity)) * self.radius
        self.arrowDw = canvas.create_line(*self.coords, *(self.coords + arrow_v), fill=kwargs.get("linefill",  "red"), width=2, arrow=LAST)
    
    def clean(self, canvas):
        """
        Removes the circle from the canvas.

        Parameters:
            canvas (`tk.Canvas <https://tkdocs.com/shipman/canvas.html>`_): Canvas object for cleaning.

        """
        canvas.delete(self.circleDw)
        canvas.delete(self.arrowDw)

class MainWindow:
    """MainWindow for Brownian motion simulation using tkinter.
    """
    def __init__(self, args, timDelta=0.001):
        self.maxX = args.canvas_size + 5
        self.maxY = args.canvas_size + 5
        self.dt = timDelta
        self.win = Tk(className="brownian")

        self.canvas = Canvas(self.win, width=self.maxX, height=self.maxY, background="white")

        self.canvas.create_rectangle(0,0,5, self.maxY-5, fill='grey', outline='black', width=2)
        self.canvas.create_rectangle(0, self.maxY-5 , self.maxX-5, self.maxY, fill='grey', outline='black', width=2)
        self.canvas.create_rectangle(self.maxX-5, 5 , self.maxX, self.maxY, fill='grey', outline='black', width=2)
        self.canvas.create_rectangle(5,0,self.maxX, 5, fill='grey', outline='black', width=2)
        self.canvas.pack()

        start_button = Button(self.win, text="Randomize Velocity", command=self.set_random_velocity)
        start_button.pack()

        self.circle = Circle(
            coords=np.array([args.canvas_size/2, args.canvas_size/2]),
            diameter=args.diameter,
            theta=args.angular_velocity,
            stop_and_rotate=args.stop_and_rotate)

        self.start()

        self.win.mainloop()

    def start(self):
        self.circle.draw(self.canvas, fill="black")
        self.canvas.update()
        self.win.after(100, self.step)

    def step(self):
        self.circle.clean(self.canvas)
        self.circle.move(self.dt, self.maxX-5, self.maxY-5)
        self.circle.draw(self.canvas, fill="black")
        self.canvas.update()
        self.win.after(int(self.dt*1000), self.step)

    def set_random_velocity(self):
        """
        Set a random velocity for the circle on button click.
        Internally calls :func:`Circle.get_random_velocity`
        """
        vel = self.circle.get_random_velocity()
        self.circle.velocity = vel

def main():

    parser = argparse.ArgumentParser(description="Brownian Motion")

    parser.add_argument("-d", "--diameter", type=int, default=30, help="Diameter of the point/circle")
    parser.add_argument("-s", "--canvas_size", type=int, default=400, help="Size of the Canvas")
    parser.add_argument("-w", "--angular_velocity", type=int, default=3, choices=list(range(1,10)) + [20], help="Rotational Velocity (good value is between 1 to 10. Don't trust me? Try `-w 20`)")
    parser.add_argument("--seed", type=int, default=42, help="Random Seed")
    parser.add_argument("--stop_and_rotate", action='store_true', default=False, help="To rotate the robot for random duration without having linear velocity")
    args = parser.parse_args()
    # TODO: check if diameter is larger than canvas
    np.random.seed(args.seed)
    MainWindow(args)

if __name__ == '__main__':
    main()