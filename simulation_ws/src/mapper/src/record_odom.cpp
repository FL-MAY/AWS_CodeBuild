//
// Created by rock-trl on 8/28/19.
//

#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

using namespace std;
using std::ios;

int main(int argc, char** argv) {
	ROS_INFO("Collecting data...");

	std::ofstream f("/home/rock-trl/workspace/scrubber_test/mapper/ROS_bag/odom.txt", ios::app | ios::out);

	rosbag::Bag bag;
	bag.open("/home/rock-trl/workspace/scrubber_test/mapper/ROS_bag/test.bag");

	for (rosbag::MessageInstance const m: rosbag::View(bag)) {
		nav_msgs::Odometry::ConstPtr i = m.instantiate<nav_msgs::Odometry>();
		if (i != nullptr) {
			auto x = i->pose.pose.position.x;
			auto y = i->pose.pose.position.y;
			tf::Quaternion q(i->pose.pose.orientation.x, i->pose.pose.orientation.y,
			                 i->pose.pose.orientation.z, i->pose.pose.orientation.w);
			tf::Matrix3x3 m(q);
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);
			f << x << " " << y << " " << yaw << std::endl;
		}
	}

	bag.close();
	f.close();

	return 0;
}
