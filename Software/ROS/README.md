# Dogbot ROS Components

DogBot is integrated with ROS for ease of integration with your own systems.  Refer to the main [Dogbot repo] instructions for initial setup.

# Operation

## Initial Operation and Controllers

Interaction with the hardware requires the firmware components found in the [Firmware] repo, and the motors reqiure 'homing' via the firmware UI.  Once this is done, basic components can be run up using:

```bat
cd Software/ROS
catkin build
source devel/setup.bash
roslaunch dogbot_control dogbot_hardware.launch
```

12 controllers are instantiated, e.g. *front_right_knee_joint_controller*.  The [ros_control](http://wiki.ros.org/controller_manager) package is used to spawn and interact with ROS controllers, with the outline code beign built around Dave Coleman's [boilerplate] examples.

Currently the hardware interface only operates for position controllers, and only accepts absolute positions, not relative ones (hence the motors must be homed).

## Control via Topics, examples

Show the current joint states: ```rostopic echo -n 1 /dogbot/joint_states```

Sample output:
```
header: 
  seq: 307571
  stamp: 
    secs: 1517397784
    nsecs: 684705073
  frame_id: ''
name: [front_right_roll_joint, front_right_pitch_joint, front_right_knee_joint, front_left_roll_joint,
  front_left_pitch_joint, front_left_knee_joint, back_right_roll_joint, back_right_pitch_joint,
  back_right_knee_joint, back_left_roll_joint, back_left_pitch_joint, back_left_knee_joint]
position: [0.10201127827167511, 0.09683401137590408, 0.8471538424491882, 0.05003453932963618, -0.0013422536430880427, -0.010162778198719025, 0.00364325987175107, -0.002109255874529481, -0.012463783845305443, 0.006903018802404404, 0.013039035722613335, 0.0074782706797122955]
velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
acceleration: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

Show just front_right_knee_joint position: ```rostopic echo -n 1 /dogbot/joint_states/position[2]```

Set a joint position:
```rostopic pub /dogbot/front_right_knee_position_controller/command std_msgs/Float64 "data: -1.0" -1```


[boilerplate]: https://github.com/davetcoleman/ros_control_boilerplate
[Dogbot repo]: https://github.com/craftit/Dogbot
[ROS]: http://www.ros.org
[Firmware]: https://github.com/craftit/BMC2-Firmware
