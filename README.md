# Dogbot

[DogBot] is a quadruped robot developed by [React AI] as a platform for researching robotics, AI and more.

The Dogbot repository contains CAD designs, along with software for higher-level systems for control and operation of the [React AI] DogBot robot via [ROS].  It utilises the lower-level motor control firmware in the corresponding [Firmware] repository for interaction with the physical DogBot.

# Installation

This code has been tested for installation and operation on Ubuntu Linux.  It is recommended, though not essential, that the Dogbot and [Firmware] repositories are installed side by side if you wish to interact with a physical DogBot or a test rig.

[ROS] must be installed; the code has been tested against the Kinetic release.  The repository contains a [catkin](http://wiki.ros.org/catkin) workspace under [Software/ROS](./Software/ROS), which should be built with `catkin build`

# Operation via Firmware and ROS

The motors must be ‘homed’, i.e. know their absolute position, before they can be controlled via the ROS driver, hence they must first be controlled via the UI as described in the [Firmware] repo.

To get started with ROS control, build and source the code then run `roslaunch dogbot_control dogbot_hardware.launch`

If you have any problems with the contents of either repository, or have a question about [DogBot], please feel free to [contact us](https://www.reactai.com/)

# Contributing

If you wish to contribute please fork the project on GitHub.  More details on contributing to follow.

# License

details to follow

[DogBot]: https://www.reactai.com/dog-bot/
[React AI]: https://www.reactai.com
[ROS]: http://www.ros.org
[Firmware]: https://github.com/craftit/BMC2-Firmware
