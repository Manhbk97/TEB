# local_planner

`local_planner` is a ROS package designed for local path planning and obstacle avoidance in robotic navigation. It generates safe and efficient trajectories for robots in dynamic environments.

## Features

- Local trajectory generation based on sensor data
- Obstacle detection and avoidance
- Configurable planning parameters
- Integration with ROS navigation stack

## Dependencies

- ROS (Melodic/Noetic)
- `geometry_msgs`
- `nav_msgs`
- `sensor_msgs`
- `tf`
- `roscpp`
- `rospy`

## Installation

Clone the repository into your catkin workspace and build:

```bash
cd ~/catkin_ws/src
git clone <repository_url>
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Usage

Launch the local planner node:

```bash
roslaunch local_planner local_planner.launch
```

## Configuration

Parameters can be set in the `config/` directory. Adjust planner settings to fit your robot and environment.

## Nodes

- **local_planner_node**: Main node for local planning and control.

## Topics

- **Subscribed:** `/scan`, `/odom`, `/goal`
- **Published:** `/cmd_vel`, `/local_plan`

## License

MIT License

## Maintainers

- [Your Name](mailto:manh@rgt.kr)
