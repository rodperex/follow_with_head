# follow_with_head

This repository has been created to evaluate the impact of system configuration on a real robot running ROS 2, such as the PAL Robotics TiaGo, with a focus on its pan and tilt head movements. The goal is to assess how different configurations affect the robot's performance, particularly in terms of reactiveness when tracking a moving colored ball.

## Installation instructions
TBD...

## Running the experiment

### Making the robot following a colored object with its head
TBD...

### Making another robot moving in circles with the colored object to be followed

Just publish a twist command:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear:
  x: 0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.2" -r <desired_rate>
```
