# Dogbot

[DogBot] is a quadruped robot developed by [React AI] as a platform for researching robotics, AI and more.

The Dogbot repository contains CAD designs, along with software for higher-level systems for control and operation of the [React AI] DogBot robot via [ROS].  It utilises the lower-level motor control firmware in the corresponding [Firmware] repository for interaction with the physical DogBot.

# Installation

This code has been tested for installation and operation on Ubuntu Linux.

[ROS] must be installed; the code has been tested against the Kinetic release.  The repository contains a [catkin](http://wiki.ros.org/catkin) workspace under [Software/ROS](./Software/ROS), which should be built with `catkin build`

# Operation via Firmware and ROS

## Home the motors

The motors must be ‘homed’, i.e. know their absolute position, before they can be controlled via the ROS driver, hence they must first be controlled via the UI as described in the [Firmware] repo.

**Note:** complete the Firmware UI steps before starting the ROS components.  The hardware interface node in ROS will send conflicting signals if it is operational while the UI tries to alter the motor positions.

## Quick Start

To get started with ROS control, build and source the code then run `dogbot_hardware.launch`

```bat
cd Software/ROS
catkin build
source devel/setup.bash
roslaunch dogbot_control dogbot_hardware.launch
```

This launches a hardware interface node of type dogbot_control/dogbot_hw_main, which uses the [ros_control](http://wiki.ros.org/controller_manager) package to spawn and interact with ROS controllers.  This provides a framework for different controller types, as well as standardising interactions with Gazebo and RViz.

If you are controlling other equipment you may require a different launch file, such as testrig_hardware.launch

## Further ROS Control

The motors can now be controlled by interacting directly with ROS topics or via ROS nodes. Further details are in the ROS directory [readme file](./Software/ROS)

# Contributing

If you have any problems with the contents of either repository, or have a question about [DogBot], please feel free to [contact us](https://www.reactai.com/)

If you wish to contribute please fork the project on GitHub.  More details on contributing to follow.

# License

details to follow

[DogBot]: https://www.reactai.com/dog-bot/
[React AI]: https://www.reactai.com
[ROS]: http://www.ros.org
[Firmware]: https://github.com/craftit/BMC2-Firmware
