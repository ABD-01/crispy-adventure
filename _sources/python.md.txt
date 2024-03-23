# Python Challenge

:::{toctree}
---
maxdepth: 2
---

brownian
:::

This is my solution to the [python challenge](_static/GSoC-2024%20Python%20test.pdf) of simulating brownian motion for a robot.

```{raw} html
<p class="centered">
  <video width="50%" autoplay muted loop>
    <!-- <source src="https://drive.google.com/file/d/1OSvqhp7e98E5Dyr5qG3soNnvlr28CHbY/view?usp=sharing" type="video/mp4" /> -->
    <source src="_static/brownian.mkv" type="video/mp4" />
    Your browser does not support the video tag.
  </video>
</p>
```

## Usage

Download the file [brownian_motion-1.0.0-py3-none-any.whl](https://github.com/ABD-01/crispy-adventure/releases/download/1.0.0/brownian_motion-1.0.0-py3-none-any.whl) and install it in your Python environment using pip. <br> (Visit [releases](https://github.com/ABD-01/crispy-adventure/releases/tag/1.0.0) section if unable to download.)

```bash
$ pip install brownian_motion-1.0.0-py3-none-any.whl
```

You can now invoke the interface using the command `brownian`.

```
$ brownian --help
usage: brownian [-h] [-d DIAMETER] [-s CANVAS_SIZE] [-w ANGULAR_VELOCITY]
                [--seed SEED] [--stop_and_rotate]

Brownian Motion

options:
  -h, --help            show this help message and exit
  -d DIAMETER, --diameter DIAMETER
                        Diameter of the point/circle
  -s CANVAS_SIZE, --canvas_size CANVAS_SIZE
                        Size of the Canvas
  -w ANGULAR_VELOCITY, --angular_velocity ANGULAR_VELOCITY
                        Rotational Velocity (good value is between 1 to 10.
                        Don't trust me? Try `-w 20`)
  --seed SEED           Random Seed
  --stop_and_rotate     To rotate the robot for random duration without
                        having linear velocity
```

```{youtube} 7loXQsexMGU
---
align: center
---

```

## Why Tkinter?

Firstly, Tkinter is a python standard library for GUIs so I did not break any rules. 

1. My experience with pygame was kindof unsatisfactory last time I used it hence, decided not to go with it for this application.
2. I have made animations in Matplotlib, but it was my personal preference to choose tkinter.


## The Ambiguous Requirement

> Mostly the robot would keep moving forward. On collision with the boundary, the robot would rotate for a random duration and then keep moving forward in the set direction.

This is open to different interpretation of the collision behaviour. What happens to linear velocity? 

a) Does the robot keep moving forward while rotating? 

b) Or does it looses the linear velocity, rotates for a duration and go back to moving forward?

Hence I have implemented such that option `a` is default but also option `b` can be selected with 

```
brownian --stop_and_rotate
```

## Use of AI

Why add this section? Because I read a discussion where [@jmplaza](https://github.com/jmplaza) mentioned how selection process is more rigorous in these ChatGPT times.
<br>
I have not used any assitance of AI tools like ChatGPT, co-pilot, etc for the programming part of this challenge.
<br>
What I have used ChatGPT is for the docstrings you will see in the next page.
