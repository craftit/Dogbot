# dogbot_gazebo

Controls operation of simulated DogBot(s) in gazebo.  The robot description is in the dogbot_description package.

Control software is in dogbot_control, as the same software drives the simulation and the hardware

# Quick start

<pre>
cd Software/ROS
catkin build
source devel/setup.bash
roslaunch dogbot_gazebo gztest.launch
</pre>

Then to send commands to the joints and make the simulated DogBot move its legs around, in roughly a walking-on-the-spot motion, switch to another window and type:

<pre>
cd src/dogbot_gazebo/bags/
rosbag play -l demo1.bag
</pre>

# Launch files

Top-level files start with gz

Other files are components, intended to be included into the higher-level files (esp. to set the namespace correctly)

# Prerequisites

Ensure you have gazebo-ros control packages installed, http://wiki.ros.org/gazebo_ros_control

<pre>sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control</pre>
