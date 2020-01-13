# Scrubber Gazebo Package
---
### 主要功能  
---
*    启动Scrubber的Gazebo仿真环境  
     > launch->scrubber_world.launch  
     目前使用的是turtlebot3_gazebo库里的house模型  
     TODO: 根据后续仿真需求搭建世界模型  
---
*    使用Cartographer建图
     > launch->scrubber_cartographer.launch  
     目前依赖laser + ODOM建图
     Requirement: 
     *    robot description(urdf)
     *    robot state publisher
     *    sensor data(simulation/real world)
     Note: 对于仿真下建图，需要同时运行仿真环境获取仿真环境下的  
     传感器数据与机器人STATE（scrubber_world.launch）
