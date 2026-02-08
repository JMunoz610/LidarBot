# PID Controller Robot (January 2026)

Building off the WallFollowingPID (insert link).

---

## Layout of Everything

### lidar_bot_bringup
- config: bridge between Gazebo and ROS2
- launch: gazebo launch file (launches gazebo and rviz) and rviz launch file
- worlds: contains the gazebo world

### lidar_bot_control
- src: contains random_walk.cpp and wall_following.cpp

### lidar_bot_description
- rviz: contains the rviz configuration file
- urdf: contrains the robot's urdf

## Running the wall_following algorithm

To launch the gazebo world with the robot:

```bash
ros2 launch lidar_bot_bringup gazebo.launch.xml
```

(insert image of gazebo and rviz, can also show how to visualize lidar)

In a separate terminal launch the wall_following.cpp file

```bash
ros2 run lidar_bot_control wall_following.cpp
```

## Running the random_walk algorithm

To launch the gazebo world with the robot:

```bash
ros2 launch lidar_bot_bringup gazebo.launch.xml
```

In a separate terminal launch the random_walk.cpp file

```bash
ros2 launch lidar_bot_control random_walk.cpp
```
