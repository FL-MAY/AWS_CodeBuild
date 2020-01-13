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

using namespace std;
using std::ios;

using pose_2d_type = struct {
	double x;
	double y;
	double th;
};

geometry_msgs::PoseWithCovarianceStamped amcl_pose;

void amcl_pose_Callback(const geometry_msgs::PoseWithCovarianceStamped &msg) {
	amcl_pose = msg;
//	std::cout << "AMCL pose: " << amcl_pose.pose.pose.position.x << "  " << amcl_pose.pose.pose.position.y << "  " << amcl_pose.pose.pose.orientation.z << std::endl;
}

bool pose_updated(pose_2d_type l, pose_2d_type r) {
	auto p_th = 0.1;
	auto o_th = 3.14 / 18;
	p_th *= p_th;
	o_th *= o_th;

	auto dis_x  = l.x - r.x;
	auto dis_y  = l.y - r.y;
	auto dis_p  = dis_x * dis_x + dis_y * dis_y;

	auto dis_o = (l.th - r.th) * (l.th - r.th);

	if (dis_p > p_th || dis_o > o_th)
		return true;

	return false;
}

void write_data(double x, double y, double yaw) {
	std::ofstream f("/home/rock-trl/workspace/scrubber_test/navigator/pose_array.txt", ios::app | ios::out);
	f << x << " " << y << " " << yaw << std::endl;
	f.close();
}

int main(int argc, char** argv) {
	ROS_INFO("Collecting data...");

	ros::init(argc, argv, "record_path");
	ros::NodeHandle nh;
	ros::Subscriber amcl_listener = nh.subscribe("amcl_pose", 1, &amcl_pose_Callback);

	pose_2d_type last_pose;

	ros::Rate r(5);
	while (ros::ok()) {
		tf::Quaternion q(amcl_pose.pose.pose.orientation.x, amcl_pose.pose.pose.orientation.y,
		                 amcl_pose.pose.pose.orientation.z, amcl_pose.pose.pose.orientation.w);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		auto current_pose = pose_2d_type {amcl_pose.pose.pose.position.x, amcl_pose.pose.pose.position.y, yaw};

		if (pose_updated(current_pose, last_pose)) {
//			std::cout << "in AMCL pose: " << amcl_pose.pose.pose.position.x << "  " << amcl_pose.pose.pose.position.y
//			          << "  " << amcl_pose.pose.pose.orientation.z << "  " << amcl_pose.pose.pose.orientation.w << std::endl;
			write_data(current_pose.x, current_pose.y, current_pose.th);
			last_pose = current_pose;
		}

		r.sleep();
		ros::spinOnce();
	}

	return 0;
}
