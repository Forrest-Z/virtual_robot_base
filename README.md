暂时只测试了BR201的型号，以及在梦工厂5号楼、6号楼、7号楼通过了 __进入电梯__、__跨楼层导航__ 和 __跨楼层的装卸箱__

# 替换掉的节点：
1. 底盘节点

2. 装卸箱节点

3. 梯控节点

3. 激光雷达节点（或其他传感器节点）


# 禁用节点：
1. openCV二维码节点

2. 日志节点

3. 音频节点（可以正常开启使用，但测试时觉得太吵，就禁用了）

# 虚拟节点订阅和发布的主题：
## 虚拟底盘节点 [/virtual_robot_base_node]

### Publications: 
 * /odom [nav_msgs/Odometry]
 * /tf [tf2_msgs/TFMessage]

### Subscriptions: 
 * /cmd_vel_mux/output/cmd_vel [geometry_msgs/Twist]
 * /robot_base/sensors [cti_msgs/BaseSensors]


## 虚拟梯控节点 [/virtual_elevator_control_node]
### Publications: 
 * /lift_state [cti_msgs/LiftState]

### Subscriptions: 
 * /elevator_cmd [cti_msgs/ElevatorCmd]
 * /robot_state [cti_msgs/BuildingRobotState]


## 虚拟激光雷达节点 [/virtual_laser_sensor]
### Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /scan [sensor_msgs/LaserScan]

### Subscriptions: 
 * /navigation_map [nav_msgs/OccupancyGrid]


## 虚拟装卸箱节点 [/virtual_docking_node]
### Publications: 
 * /box_mount_state [cti_msgs/BoxMountState]
 * /cmd_vel_mux/input/docking [geometry_msgs/Twist]
 * /robot_base/sensors [cti_msgs/BaseSensors]

### Subscriptions: 
 * /box_mount_cmd [cti_msgs/BoxMountCmd]

# 暂未实现IMU和超声波的模拟