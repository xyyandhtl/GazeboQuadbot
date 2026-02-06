# Gazebo Go2 Quadbot for 3D Autonomous Navigation

Forked from [gazebo_go2_simulation](https://github.com/dfl-rlab/gz_quadbot.git)
Livox lidar simulation: 
- [official_gazebo9_ros1](https://github.com/Livox-SDK/livox_laser_simulation)
- [gazebo11_ros2_version](https://github.com/inkccc/mid360_simulation)

# Additional Features
- **add gps sensor**: before running, set latitude_deg / longitude_deg / elevation / heading_deg in xx.world to set the gps origin
- **add livox lidar**: 
- **add more worlds**: 

| World | Scene Type |
|-------|------|
|slope_with_pillar_2 | outdoor, closure, multi floor, no stairs |
|standardrobots_factory | indoor, closure, multi floor, stairs |



# Topics List
| Topic | Type |
|-------|------|
| /base_to_footprint_pose | geometry_msgs/msg/PoseWithCovarianceStamped |
| /body_pose | geometry_msgs/msg/Pose |
| /clock | rosgraph_msgs/msg/Clock |
| /cmd_vel | geometry_msgs/msg/Twist |
| /dynamic_joint_states | control_msgs/msg/DynamicJointState |
| /foot | visualization_msgs/msg/MarkerArray |
| /foot_contacts | champ_msgs/msg/ContactsStamped |
| /gps/data | sensor_msgs/msg/NavSatFix |
| /gps_plugin/vel | geometry_msgs/msg/Vector3Stamped |
| /imu/data | sensor_msgs/msg/Imu |
| /joint_group_effort_controller/controller_state | control_msgs/msg/JointTrajectoryControllerState |
| /joint_group_effort_controller/joint_trajectory | trajectory_msgs/msg/JointTrajectory |
| /joint_group_effort_controller/state | control_msgs/msg/JointTrajectoryControllerState |
| /joint_group_effort_controller/transition_event | lifecycle_msgs/msg/TransitionEvent |
| /joint_states | sensor_msgs/msg/JointState |
| /joint_states_controller/transition_event | lifecycle_msgs/msg/TransitionEvent |
| /odom/raw | nav_msgs/msg/Odometry |
| /parameter_events | rcl_interfaces/msg/ParameterEvent |
| /performance_metrics | gazebo_msgs/msg/PerformanceMetrics |
| /robot_description | std_msgs/msg/String |
| /rosout | rcl_interfaces/msg/Log |
| /tf | tf2_msgs/msg/TFMessage |
| /tf_static | tf2_msgs/msg/TFMessage |
| /velodyne_points | sensor_msgs/msg/PointCloud2 |
