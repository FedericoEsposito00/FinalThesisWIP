# Cable Suspended Robots

Environment and source code to simulate cable-suspended dual-arm manipulators developed in the paper:

*[G. D'Ago](), [M. Selvaggio](http://wpage.unina.it/mario.selvaggio/index.html), [A. Suarez](), [F. J. Gañán Onieva](), [L. R. Buonocore](), [M. Di Castro](), [V. Lippiello](http://wpage.unina.it/lippiell/), [A. Ollero](https://grvc.us.es/aollero/home_en.html), [F. Ruggiero](http://www.fabioruggiero.name/web/index.php/en/) "Modelling and identification methods for simulation of cable-suspended dual-arm robotic systems", in IEEE Robotics and Automation Letters, submitted.* 


[![Cable Suspended Robots](gazeboCranebot.png)]()


# Instructions

The code was tested with Ubuntu 20.04, and ROS Noetic. Different OS and ROS versions are possible but not supported.

### Install dependencies

`sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers`

### Compile

`catkin_make UnforcedOscillationLicas UnforcedOscillationCranebot ArmsInducedOscillationCranebot`

### Run simulation CRANEbot

To start the simulation of the cable-suspended robot hanging from a crane called CRANEbot, launch:

`roslaunch cable-suspended-robots-ros-pkg cranebot.launch`

Once the simulation has started, in another terminal type the following command to generate an arms-induced oscillation around the vertical axis:

`rosrun cable-suspended-robots-ros-pkg armsInducedOscillationCranebot.launch`

while to generate an unforced oscillation around the Y axis just run:

`rosrun cable-suspended-robots-ros-pkg unforcedOscillationCranebot.launch`


### Run simulation LiCAS

To launch the simulation of the cable-suspended robot hanging from a drone called LiCAS A1, type:

`roslaunch cable-suspended-robots-ros-pkg licasa1.launch`

In another terminal run the following command to generate an unforced oscillation in a X-Y combined plane:

`rosrun cable-suspended-robots-ros-pkg unforcedOscillationLicas.launch`
