# Instructions to Run:

## Libraries:
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import matplotlib.patches as mpatches
import math
import sys
import timeit
import matplotlib.path as mplPath

## Code

- Run ***astar.py*** file: ```python astar.py```

This will prompt the user for the inputs shown below: 

Robot dimension: This is the radius of the mobile robot (this will expand the radius of obstacle space in red accordingly)
Obstacle clearance: Clearance area from obstacles, shown in red around the obstacles
Start X (integer): Initial x start coordinate
Start Y (integer): Initial y start coordinate
Start theta (degrees): Initial angle placement of mobile robot
Goal X (integer): x goal coordinate
Goal Y (integer): y goal coordinate
Step Size (1-10): Step size of the robot while it explores the path

## Visualization

After running the code and providing inputs, the following outputs are provided:
- Obstacles will appear in black
- Red spaces are due to robot's dimension and clearance
- Unexplored points in white
- Finally, zoom in on the explored path which will be mostly seen as green at first. (Scaling)
- After zooming in, we can find three colors:
  1. Yellow dots: These are the explored nodes
  2. Green: Chosen nodes of A* (Backtracking)
  3. Blue: Connects the chosen nodes to see the path (visual aid)

![astar](https://user-images.githubusercontent.com/24211929/159390771-bf0cce5e-5255-43c1-9010-149fa303ebdf.gif)

