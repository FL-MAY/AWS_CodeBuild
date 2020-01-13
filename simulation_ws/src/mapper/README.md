Get start with mapper
===
Install cartographer_ros
---
* Install wstool and rosdep<br>
`sudo apt-get update`<br>
`sudo apt-get install -y python-wstool python-rosdep ninja-build`<br>
* Create a new workspace in 'carto_ws'<br>
`mkdir ~/carto_ws`<br>
`cd carto_ws`<br>
`wstool init src`<br>
* Merge the cartographer_ros.rosinstall file and fetch code for dependencies<br>
`wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall`<br>
* Modify install files to aviod network unreachable<br>
`cd src`<br>
`mv .rosinstall rosinstall`<br>
`gedit rosinstall`<br>
Replace with<br>
"local-name: ceres-solver<br>
uri: https://github.com/ceres-solver/ceres-solver.git"<br>
`mv rosinstall .rosinstall`<br>
`cd ..`<br>
* Continue<br>
`wstool update -t src`<br>
`rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y`<br>
* Build and install<br>
`catkin_make_isolated --install --use-ninja`<br>
`source install_isolated/setup.bash`<br>

Use cartographer mapping
---
* Open a simulation environment, read navigator/README.md<br>
* Open terminal 1. Input `roslaunch mapper mapper.launch`<br>
* Open terminal 2. Input `python RoboScrub_Nav/src/navigator/teleop/navigator_teleop_key`<br>
Control the robot by the program hint.<br>
* Save map: open terminal 3. Input `rosrun map_server map_saver -f ~/map`<br>

Incrementally mapping
---
* Purpose: support mapping at existing map.<br>
* After save map for a new mapping, save cartographer pbstream.<br>
`rosservice call /write_state "{filename: '/home/xxx/map.bag.pbstream'}"`<br>
* `roslaunch mapper mapper.launch pure_localization:=true`<br>
* For efficiency, we suggest you set an initial pose for the incrementally mapping process.<br>
To do this, first you need to check the current trajectory number and finish it.<br>
`rostopic echo /trajectory_node_list`<br>
Finish the current trajectory that is not in FROZEN status.<br>
`rosservice call /finish_trajectory "trajectory_id: x"`<br>
Then we can start a new trajectory with a known initial pose.<br>
`rosservice call /start_trajectory "configuration_directory: '/home/xxx/mapper/config'
 configuration_basename: 'scrubber_2d.lua'
 use_initial_pose: true
 initial_pose:
    position: {x: xx, y: xx, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: xx, w: xx}
  relative_to_trajectory_id: 0"`<br>
* The pose may be a little inaccurate, so we can wait the robot for a moment till localized a correct pose.<br>
* Move the robot for incrementally mapping.<br>
* When finish, save the new map and pbstream file.<br>

Debug mapping
---
* Purpose: simulate mapping process from ros bag files.<br>
* How to record bag files:<br>
You can simply record all topic by command `rosbag record -a`, but we perfer only record useful topic by command `rosbag record tf_static tf odom scan scan1`, needless message will confuse cartographer.<br>
* How to simulate:<br>
First of all, we can put bag files in ${HOME}/workspace/RoboScrub_Nav/src/mapper/ROS_bag/, otherwise we can change the log dir in code.<br>
Then we need filter bag files to delete map related tf message, cause it's generate by cartographer on real machine.<br>
It's like `rosbag filter origin.bag filter.bag "topic!='tf' or m.transforms[0].header.frame_id!='map'"`<br>
At last, simulate command is like `roslaunch mapper simu_backpack.launch bag_filename:=${HOME}/workspace/RoboScrub_Nav/src/mapper/ROS_bag/filtered.bag`.<br>
You can modify the replay rate in simu_backpack.launch.

Automatically mapping
---
* Purpose: for easy mapping.<br>
* Prepare: you need this node: https://github.com/TonyRockrobo/m-explore <br>
* How to use: <br>
Simplyly type `roslaunch mapper mapperex.launch` instead of previous command. <br>
* One step more: <br>
If the robot cannot travel to somewhere new in the map, you can stop the explore process by `rosnode kill /explore` <br>
You can then use `2D navigation` button or keybroad program control robot to the position you want. <br>
Wanna explore again? Type `roslaunch explore_lite explore.launch`
