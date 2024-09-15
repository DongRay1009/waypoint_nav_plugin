# waypoint_nav_plugin
This is a rviz tool plugin. if you want to use it, you need to configure gazebo simulation environment first and configure nav.launch in this package.
I implemented it so that this plugin automatically generates a waypoint.txt in the home directory after selecting the target point. And use this waypoint.txt to realize the traversal of all path points.But there is a problem that is still not solved: there is no icon for the rviz tool plugin. I've been working on this for a couple of hours and it still doesn't work.

Or, since I am using the simulation environment of wpr_simulation in my study, you can compile wpr_simulation, wpb_home and this package in the same workspace directly from GitHub.

1. Select waypoint and create waypoint.txt file<br>
 ```
roslaunch pose_get_plugin waypoint_making.launch
//Generally, waypoint.txt is saved to the home directory<br>
 ```
3. Implementation of navigation<br>
 ```
roslaunch wpr_simulation wpb_stage_robocup.launch <br>
//Run the simulation environment, which can be any other gazebo environment you have.<br>
roslaunch pose_get_plugin nav.launch<br>
//If you are using a different simulation environment, you may need to modify this file<br>
rosrun pose_get_plugin multi_waypoint_nav<br>
//After running this node the robot will start to navigate<br>
 ```
About the structure of the rviz plugin tool package, the option to add a new tool in rviz will appear for this plugin only if plugin_description.xml, plugin_name.cpp and plugin_name.h are configured.<br>
 ```
my_plugin/<br>
│<br>
├── CMakeLists.txt<br>
├── package.xml<br>
├── plugin_description.xml<br>
│<br>
├── include/<br>
│   └── package_name/<br>
│       └── plugin_name.h<br>
│<br>
├── src/<br>
│   └── plugin_name.cpp<br>
│<br>
├── rviz/<br>
│<br>
└── launch/<br>
 ```
