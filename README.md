# follow_with_head

This repository has been created to evaluate the impact of system configuration on a real robot running ROS 2, such as the PAL Robotics TiaGo, with a focus on its pan and tilt head movements. The goal is to assess how different configurations affect the robot's performance, particularly in terms of reactiveness when tracking a moving colored ball.

## Installation instructions

```bash
cd ~/ros2_ws/src
git clone https://github.com/rodperex/follow_with_head.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

## Parameters

Use of intra-process communitcation and real-time, among other parameters, can be configured in ``follow_with_head/config/params.yaml``.

## Running the experiments

To launch each node in its own executor, simply run the following commands in three separate terminals

```bash
ros2 launch follow_with_head hsv_filter.launch.py
ros2 launch follow_with_head depth_estimator.launch.py
ros2 launch follow_with_head head_follow_controller.launch.py
```

Or run this command:

```bash
ros2 launch follow_with_head follow_with_head.launch.py
```

In case you want to run all nodes within the same executor:

```bash
ros2 launch follow_with_head whole.launch.py
```
