#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <mbf_msgs/ExePathAction.h>
#include <mbf_msgs/GetPathAction.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <std_msgs/String.h>

typedef actionlib::SimpleActionClient<mbf_msgs::ExePathAction> PathClient; // A client of Exe Path Action Server

geometry_msgs::PoseWithCovarianceStamped amcl_pose;
bool get_initial_pose = false;

void amcl_pose_Callback(const geometry_msgs::PoseWithCovarianceStamped &msg) {
	amcl_pose = msg;
	get_initial_pose = true;
	std::cerr << "AMCL pose: " << amcl_pose.pose.pose.position.x << "  " << amcl_pose.pose.pose.position.y << "  " << amcl_pose.pose.pose.orientation.z << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_path");
    ros::NodeHandle nh;

    ros::Subscriber amcl_listener = nh.subscribe("amcl_pose", 1, &amcl_pose_Callback);

    while (!get_initial_pose) {
		ros::spinOnce();
    }

    PathClient pc("move_base_flex/exe_path", true); // true doesnt need ros::spin

     while(!pc.waitForServer(ros::Duration(5.0))){
         ROS_INFO("Waiting for Move Base server to come up");
     }

    mbf_msgs::ExePathGoal target_path_;

    nav_msgs::Path path_;

    //TODO: Get AMCL pose
	double x   = amcl_pose.pose.pose.position.x;
	double y   = amcl_pose.pose.pose.position.y;
	double th  = amcl_pose.pose.pose.orientation.z;
	double vx  = 0.1;
	double vy  = -0.1;
	double vth = 0.1;

    while (th < 6.28) {
	    double dt       = 0.1;
	    double delta_x  = (vx * cos(th) - vy * sin(th)) * dt;
	    double delta_y  = (vx * sin(th) + vy * cos(th)) * dt;
	    double delta_th = vth * dt;

	    x += delta_x;
	    y += delta_y;
	    th += delta_th;

	    geometry_msgs::PoseStamped this_pose_stamped;
	    this_pose_stamped.pose.position.x = x;
	    this_pose_stamped.pose.position.y = y;

	    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(th);
	    this_pose_stamped.pose.orientation.x = goal_quat.x;
	    this_pose_stamped.pose.orientation.y = goal_quat.y;
	    this_pose_stamped.pose.orientation.z = goal_quat.z;
	    this_pose_stamped.pose.orientation.w = goal_quat.w;

	    this_pose_stamped.header.stamp = ros::Time::now();
	    this_pose_stamped.header.frame_id = "map";
	    path_.poses.push_back(this_pose_stamped);
    }

   // Populate the Path Message
    path_.header.frame_id = "map";
    path_.header.stamp = ros::Time::now();

   // Populate the controller and path fields (Refer to ExePath.Action)
    target_path_.controller = "DWAPlannerROS";
    target_path_.path = path_;

  // Interact with Action Server
  pc.sendGoal(target_path_);

  pc.waitForResult();
  if(pc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Base moved %s", pc.getState().toString().c_str());

  else if(pc.getState() == actionlib::SimpleClientGoalState::ABORTED)
    ROS_INFO("Goal aborted");

  else
    ROS_INFO("Base failed to move for some reason");


  ros::shutdown();
  return 0;
}
