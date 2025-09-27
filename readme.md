# Default-Course-Project
Default Course Project of CS4278/CS5478 Intelligent Robots: Algorithms and Systems

In this project, the task is to:

1. Generate antipodal grasp proposals, solve IK, and pick up an object on the table.
2. Carry the object and navigate the robot through the maze. 
3. Bring the object to the target position (visualized as the green sphere). 

We provide an environment code for the robot in `pick_nav_reach.py`.

![Scene](imgs/scene.png)

# Requirement

You should implement the grasp generation, motion planning, and navigation algorithms by yourself to accomplish the task.

# Rubric (TODO)

## Grasp Generation (40%)

## Motion Planning and Grasping (20%)

## Navigation (40%)

# Installation (TODO)

1. Our environment is build on [PyBullet](https://pybullet.org/wordpress/index.php/forum-2/). Install it with pip:

```
pip3 install pybullet numpy
```

2. Clone the project repo:

```
git clone https://github.com/NUS-LinS-Lab/Pick-Nav-Reach-Default-Project.git
```

# Run the Environment 

`python run.py`

# References

- [PyBullet Quickstart Guide](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit?tab=t.0#heading=h.2ye70wns7io3).

# Acknowledgments

- YCB object models are chosen and adapted from [here](https://www.ycbbenchmarks.com/).
