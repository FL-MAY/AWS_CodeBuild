//
// Created by rock-trl on 8/2/19.
//

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

ros::Publisher pub;
float speed_ratio;

void cmd_vel_Callback(const geometry_msgs::Twist& vel) {
	geometry_msgs::Twist new_vel;
	new_vel.linear.x  = vel.linear.x * speed_ratio;
	new_vel.angular.z = vel.angular.z * speed_ratio;
	pub.publish(new_vel);
}

void speed_ratio_Callback(const std_msgs::Float32& ratio) {
	if (ratio.data < 0.0 || ratio.data > 1.0) {
		ROS_INFO("Wrong value!");
	} else {
		speed_ratio = ratio.data;
	}
}

int main(int argc, char **argv) {
	speed_ratio = 1.0;

	ros::init(argc, argv, "speed_ratio");
	ros::NodeHandle nh;

	pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	ros::Subscriber vel_sub   = nh.subscribe("mbf_cmd_vel", 10, cmd_vel_Callback);
	ros::Subscriber ratio_sub = nh.subscribe("speed_ratio", 10, speed_ratio_Callback);

	ros::spin();

	return 0;
}