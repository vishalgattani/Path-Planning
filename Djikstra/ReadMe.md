# Instructions to Run:

Code asks user for inputs for the following parameters:

- dimension 
- clearance 
- startx 
- starty 
- goalx 
- goaly 
- live animation
- video save

If the user wishes a live animation, the Opencv library will show node exploration.
If the user wishes to save the video, user requires to to input y/n.

Start node parent index = -1

Given map dimensions, each whitespace inside the map has the follwoing index: `index = j*width+i` where i and j are the nodes (whitespace pixel grid) coordinates.

The openset and closed set are dictionaries with key being the index and value being the node.

Backtracking is done by using the closed set and going from goal nodes parent till parent index of final node i.e., start node is -1.

Obstacle map is built using opencv.

Github: [Link](https://github.com/vishalgattani/Path-Planning)

