//
// Created by rock-trl on 9/24/19.
//

#include "ros/ros.h"
#include "path_coverage/GetPathInZone.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <cstdlib>
#include <tf/tf.h>
#include <iostream>
#include <fstream>

using namespace std;

int main(int argc, char **argv) {
	ros::init(argc, argv, "get_path_test");

	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<path_coverage::GetPathInZone>("getPathInZone");

	path_coverage::GetPathInZone srv;

	geometry_msgs::PoseStamped peak;
	nav_msgs::Path zone;

	//Vertical
//	peak.pose.position.x = 3.0;
//	peak.pose.position.y = 1.5;
//	zone.poses.push_back(peak);
//	peak.pose.position.x = 3.0;
//	peak.pose.position.y = -2.0;
//	zone.poses.push_back(peak);
//	peak.pose.position.x = -2.0;
//	peak.pose.position.y = -2.0;
//	zone.poses.push_back(peak);
//	peak.pose.position.x = -2.0;
//	peak.pose.position.y = 1.5;
//	zone.poses.push_back(peak);

	//Horizontal
//	peak.pose.position.x = 3.0;
//	peak.pose.position.y = 1.5;
//	zone.poses.push_back(peak);
//	peak.pose.position.x = 3.0;
//	peak.pose.position.y = -2.0;
//	zone.poses.push_back(peak);
//	peak.pose.position.x = 0;
//	peak.pose.position.y = -2.0;
//	zone.poses.push_back(peak);
//	peak.pose.position.x = 0;
//	peak.pose.position.y = 1.5;
//	zone.poses.push_back(peak);

	//Whole map
//	peak.pose.position.x = -5.5;
//	peak.pose.position.y = -2.5;
//	zone.poses.push_back(peak);

//	peak.pose.position.x = -5.5;
//	peak.pose.position.y = -2.5;
//	zone.poses.push_back(peak);
//	peak.pose.position.x = -5.5;
//	peak.pose.position.y = 1.5;
//	zone.poses.push_back(peak);
//	peak.pose.position.x = 3;
//	peak.pose.position.y = 1.5;
//	zone.poses.push_back(peak);
//	peak.pose.position.x = 3;
//	peak.pose.position.y = 8.5;
//	zone.poses.push_back(peak);
//	peak.pose.position.x = 5;
//	peak.pose.position.y = 8.5;
//	zone.poses.push_back(peak);
//	peak.pose.position.x = 5;
//	peak.pose.position.y = -2.5;
//	zone.poses.push_back(peak);

	//open record path
	std::ifstream f("/home/rock-trl/workspace/scrubber_test/navigator/pose_array_border.txt", ios::in);
	char line[1024] = {0};
	if (f) {
		while (f.getline(line, sizeof(line))) {
			geometry_msgs::PoseStamped this_pose_stamped;
			std::stringstream ss(line);
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
				zone.poses.push_back(this_pose_stamped);
//				std::cout << p[0] << " " << p[1] << " " << p[2] << " " <<std::endl;
			}
		}
		f.close();
	} else {
		ROS_INFO("No pose_array.txt");
		return 0;
	}

	srv.request.zone = zone;

	if (client.call(srv)) {
		ROS_INFO("Call service success!");
		for (auto i = 0; i < srv.response.paths.size(); i++) {
			for (auto j = 0; j < srv.response.paths[i].poses.size(); j++) {
				cout << "Path." << i + 1 << " Point." << j + 1 << " is ["
				     << srv.response.paths[i].poses[j].pose.position.x << ", "
				     << srv.response.paths[i].poses[j].pose.position.y << "]" << endl;
			}
		}
	} else {
		ROS_ERROR("Failed to call service!");
		return 1;
	}

	return 0;
}
