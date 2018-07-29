Software for a TurtleBot2 written on a ROS interface. Robot will map out unknown locations that it will be able to travel to using an A* Planning Algorithm, avoid static obstacles while using EKF Localization. Pre-set landmarks and goal are set.

Run catkin_make
Run source devel/setup.bash

For simulation, run:
	roslaunch amr_sim.launch
For running the TurtleBot2, run:
	roslaunch amr.launch
	
It takes 5 seconds for goal to be published after bringing up the robot.
