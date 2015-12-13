## ME 495 - Embedded Systems In Robotics

### Final Project - Baxter Pick and Place

#### Group Members:

    Tim Herrmann
    Mikhail Todes
    Minghe Jiang
    Yunchu Liu

#### Capabilities:

	Multiple Object Shapes
	Multiple Object Sizes
	Multiple Object Colors
	Multiple Object Orientations
	Dynamic Image Processing - can adjust object positions/orientations between picks

#### Files In Package:

* #### rightcam.py:
	This file controls and processes the right hand camera on baxter

* #### movenode.py:
This file contains the main structure of the pick place program.  The `ik_solve()` function takes a cartesian pose and returns a valid joint state position.  The `main()` function subscribesto the data published by the right cam node and uses it to direct the pick and place operation.
* #### pickplace.launch:
	Launches both the *rightcam.py* nodeand the *movenode.py* node.


* #### Image Processing:

We found our balls and squares using colour filtering image processing from the
OpenCV library. We have set defaults to look for green and red filters in the
Hue Saturation Value colour spectrum. The two filtered images are added together
using an even weighting. 

A minimum enclosing rectangle is drawn around the biggest found contour. The two 
corners with the minimum y-value are used to find a slope and thus the
orientation of the object that needs to be picked up. 

#### Cameras:
We used the camera attached to the right hand of Baxter to do the image
processing. First it is orientated at a known fixed position to scout for objects.
Once the biggest object is found, it re-orientates itself above the object. We 
adjusted the number of iterations that baxter used to recheck its position above
the object. Once it finds the object, it calculates the slope and reorientates 
itself to pick the object. 

#### Inverse Kinematics:

The  `ik_solve() ` calls an `Inverse-Kinematics(IK)` service provided by the ROS node  `/ExternalTools/<limb>/PositionKinematicsNode/ `

It takes three elements: limb, desired Quartesian coordinate and orientation of the end effector of the Baxter and will return seven joint angles that will get the arm to that position. 

Refering to Inverse Kinematics Solver Service on the Baxter API reference page:<div>http://sdk.rethinkrobotics.com/wiki/API_Reference#arm-joints </div>

The  `seed_mode ` is always set to 1 to use the user defined seeds as the initial joint angles guess for ik. The  `seed_angels ` are all very close to the current joint angles that must be in the Baxter's workplace. Since it is normally hard to get ik converge to a solution, random noise has been generated and added to the current joint angles. A list of seed angles can thus be implemented to try for at most 20 times to solve the inverse kinematics, which should be robust enough to find a valid solution. 
