Get start with navigator
===
Run navigator
---
* Download RoboticFloorScrubber project<br>
* Open terminal 1. Input `roslaunch scrubber_gazebo scrubber_world.launch`. A gazebo simulation environment will occur.<br>
* Open terminal 2. Input `roslaunch navigator navigator.launch`. Navigator and RVIZ will be load.<br>
* Use `pose_estimate` and `pose_navigation` to command the robot.<br>

Set speed ratio
---
* `rostopic pub -1 speed_ratio std_msgs/Float32 1.0`. The float number is the speed ratio between `0.0` to `1.0`.<br>

Set new initial pose (1 topic)
---
* `rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped '{header: {seq: 0, stamp: {}, frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}}'`

Range localization (2 services)
---
* Call range localization service with a given rect.<br>
`rosservice call /range_localization '{rect_min_x: 1, rect_max_x: 2, rect_min_y: 3, rect_max_y: 4}'`<br>
* Clear local maps.<br>
`rosservice call /move_base_flex/clear_costmaps`<br>
* Call nomotion update periodicly or rotate the robot manually.<br>
`rosservice call /request_nomotion_update`<br>
* Check topic amcl_pose's covariance until lower than a threshold.<br>
`rostopic echo /amcl_pose`<br>

Test follow_path
---
* `rosrun navigator test_path_node`<br>
