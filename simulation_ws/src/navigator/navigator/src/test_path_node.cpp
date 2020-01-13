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
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <iostream>
#include <fstream>

using namespace std;
using std::ios;

typedef actionlib::SimpleActionClient<mbf_msgs::ExePathAction> PathClient;
typedef actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> PointClient;

geometry_msgs::PoseWithCovarianceStamped amcl_pose;
bool get_initial_pose = false;

void amcl_pose_Callback(const geometry_msgs::PoseWithCovarianceStamped &msg) {
	amcl_pose = msg;
	get_initial_pose = true;
	std::cout << "AMCL pose: " << amcl_pose.pose.pose.position.x << "  " << amcl_pose.pose.pose.position.y << "  " << amcl_pose.pose.pose.orientation.z << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_path");
    ros::NodeHandle nh;

    ros::Subscriber amcl_listener = nh.subscribe("amcl_pose", 1, &amcl_pose_Callback);

    while (!get_initial_pose) {
		ros::spinOnce();
    }

	PointClient goTo("move_base_flex/move_base", true);
	PathClient exePath("move_base_flex/exe_path", true); // true doesnt need ros::spin

	bool goto_ol = false;
	bool exepath_ol = false;
	int patient = 0;

	ROS_INFO("Waiting for server come up...");
	while (!(goto_ol && exepath_ol) || patient++ < 10) {
		goto_ol = goTo.waitForServer(ros::Duration(2.0));
		exepath_ol = exePath.waitForServer(ros::Duration(2.0));
	}

	mbf_msgs::MoveBaseGoal target_front_;
    mbf_msgs::ExePathGoal  target_path_;

    std::vector<nav_msgs::Path> paths_;
    nav_msgs::Path path_;

    //DO: Get AMCL pose
	double x   = amcl_pose.pose.pose.position.x;
	double y   = amcl_pose.pose.pose.position.y;

	tf::Quaternion q(amcl_pose.pose.pose.orientation.x, amcl_pose.pose.pose.orientation.y,
	                 amcl_pose.pose.pose.orientation.z, amcl_pose.pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, th;
	m.getRPY(roll, pitch, th);


	//DO: read file
	std::ifstream f("/home/rock-trl/workspace/scrubber_test/navigator/pose_array.txt", ios::in);
	char line[1024] = {0};
//	double delta[3] = {0};
	bool init_flag = true;
	if (f) {
		while (f.getline(line, sizeof(line))) {
			geometry_msgs::PoseStamped this_pose_stamped;
			std::stringstream ss(line);
			std::string str(line);
			if (str == "EOP") {
				ROS_INFO("End of current path!");
				paths_.push_back(path_);
				path_.poses.clear();
				continue;
			}
			while (!ss.eof()) {
				double p[3];
				std::string token;
				for (auto i = 0; i < 3; i++) {
					ss >> token;
					p[i] = atof(token.c_str());
				}

				this_pose_stamped.pose.position.x = p[0];
				this_pose_stamped.pose.position.y = p[1];
				geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(p[2]);
				this_pose_stamped.pose.orientation.x = goal_quat.x;
				this_pose_stamped.pose.orientation.y = goal_quat.y;
				this_pose_stamped.pose.orientation.z = goal_quat.z;
				this_pose_stamped.pose.orientation.w = goal_quat.w;
				this_pose_stamped.header.stamp = ros::Time::now();
				this_pose_stamped.header.frame_id = "map";
				path_.poses.push_back(this_pose_stamped);
				std::cout << p[0] << " " << p[1] << " " << p[2] << " " <<std::endl;
			}
		}
		if (!path_.poses.empty()) {
			paths_.push_back(path_);
		}
		f.close();
	} else {
		ROS_INFO("No pose_array.txt");
		return 0;
	}

	for (auto path : paths_) {
		// Populate the Path Message
		path.header.frame_id = "map";
		path.header.stamp    = ros::Time::now();

		// Populate the controller and path fields (Refer to ExePath.Action)
//    target_path_.controller = "DWAPlannerROS";
		target_path_.controller = "TebLocalPlannerROS";
		target_path_.path       = path;

		target_front_.target_pose = target_path_.path.poses[0];

		goTo.sendGoal(target_front_);

		goTo.waitForResult();
		if (goTo.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("Goto %s", goTo.getState().toString().c_str());

		else if (goTo.getState() == actionlib::SimpleClientGoalState::ABORTED)
			ROS_INFO("Goto aborted");

		else
			ROS_INFO("Goto failed to move for some reason");


		// Interact with Action Server
		exePath.sendGoal(target_path_);

		exePath.waitForResult();
		if (exePath.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("Exe path %s", exePath.getState().toString().c_str());

		else if (exePath.getState() == actionlib::SimpleClientGoalState::ABORTED)
			ROS_INFO("Exe path aborted");

		else
			ROS_INFO("Exe path failed to move for some reason");
	}

  ros::shutdown();
  return 0;
}
