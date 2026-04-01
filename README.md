# Gazebo Go2(W) Quadbot Simulation
This repository contains the Gazebo simulation environment and URDF description for the Go2 Quadbot robot, equipped with a Livox Mid-360 LiDAR sensor and GPS module.

Forked from [gazebo_go2_simulation](https://github.com/dfl-rlab/gz_quadbot.git)

Livox lidar simulation is based on: 
- [official_gazebo9_ros1](https://github.com/Livox-SDK/livox_laser_simulation)
- [gazebo11_ros2_version](https://github.com/inkccc/mid360_simulation)

Running environment refer to: [gazebo_env_dockerfile](https://github.com/dfl-rlab/dddmr_navigation/blob/main/dddmr_docker/docker_file/Dockerfile_x64_gazebo)

# Additional Features
- **add gps sensor**: before running, set latitude_deg / longitude_deg / elevation / heading_deg in xx.world to set the gps origin
- **add livox lidar**: set lidar mounting pose in [xacro](robots/descriptions/go2_description/xacro/robot_VLP.xacro). If you don't need livox or velodyne, comment out the corresponding lines.
- **add more worlds**: see [worlds](robot_scene/worlds)

| World | Scene Type | Overview |
|-------|------|----------|
|slope_with_pillar_2 | outdoor, closure, multi floor, no stairs | ![!)](assets/scene1.png) |
|bigHHH | indoor, closure, multi room, no stairs | ![alt text](assets/scene2.png) |
|see [worlds](robot_scene/worlds) |  |  |

To use specific world with sdf resource, copy the following sdf model files
- https://github.com/leonhartyao/gazebo_models_worlds_collection/tree/master/models

to one of the following directories:
- [robot_scene/models/](robot_scene/models)
- ~/.gazebo/models/:


Launch with:
```bash
# Go2 run
ros2 launch robot_scene go2_lidar_gps.launch.py
# Go2w run
ros2 launch robot_scene go2w_lidar_gps.launch.py
# Diffbot run
ros2 launch robot_scene diffbot_lidar_gps.launch.py
```

To control the robot from keyboard, new terminal run:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


# Topics List
| Topic | Type |
|-------|------|
| /body_pose | geometry_msgs/msg/Pose |
| /cmd_vel | geometry_msgs/msg/Twist |
| /gps/data | sensor_msgs/msg/NavSatFix |
| /gps_plugin/vel | geometry_msgs/msg/Vector3Stamped |
| /livox/imu | sensor_msgs/msg/Imu |
| /livox/lidar | sensor_msgs/msg/PointCloud2 |
| /robot_description | std_msgs/msg/String |
| /tf | tf2_msgs/msg/TFMessage |
| /tf_static | tf2_msgs/msg/TFMessage |

The simulation already publishes the static tf within this robot and its sensor mounted on it:
![alt text](assets/static_tf_tree.png)