# waypoint_nav_plugin
This is a rviz tool plugin. if you want to use it, you need to configure gazebo simulation environment first and configure nav.launch in this package.
I implemented it so that this plugin automatically generates a waypoint.txt in the home directory after selecting the target point. And use this waypoint.txt to realize the traversal of all path points.

Or, since I am using the simulation environment of wpr_simulation in my study, you can compile wpr_simulation, wpb_home and this package in the same workspace directly from GitHub.

1. Select waypoint and create waypoint.txt file
roslaunch pose_get_plugin waypoint_making.launch

3. Implementation of navigation
roslaunch wpr_simulation wpb_stage_robocup.launch //Run the simulation environment, which can be any other gazebo environment you have.
roslaunch pose_get_plugin nav.launch  //If you are using a different simulation environment, you may need to modify this file
rosrun pose_get_plugin multi_waypoint_nav  //After running this node the robot will start to navigate
