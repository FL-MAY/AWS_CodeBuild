//
// Created by rock-trl on 9/24/19.
//

#include "ros/ros.h"
#include "costmap_prohibition_layer/AddProhibitionZone.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <cstdlib>

using namespace std;

int main(int argc, char **argv) {
	ros::init(argc, argv, "add_zone_test");

	ros::NodeHandle nh;
	ros::ServiceClient g_client = nh.serviceClient<costmap_prohibition_layer::AddProhibitionZone>("move_base_flex/global_costmap/costmap_prohibition_layer/AddProhibitionZone");
	ros::ServiceClient l_client = nh.serviceClient<costmap_prohibition_layer::AddProhibitionZone>("move_base_flex/local_costmap/costmap_prohibition_layer/AddProhibitionZone");

	costmap_prohibition_layer::AddProhibitionZone srv;

	geometry_msgs::PoseStamped peak;
	nav_msgs::Path zone;

	peak.pose.position.x = 5.5;
	peak.pose.position.y = -4.0;
	zone.poses.push_back(peak);
	peak.pose.position.x = 5.5;
	peak.pose.position.y = 10.0;
	zone.poses.push_back(peak);
	peak.pose.position.x = 10.0;
	peak.pose.position.y = 10.0;
	zone.poses.push_back(peak);
	peak.pose.position.x = 10.0;
	peak.pose.position.y = -4.0;
	zone.poses.push_back(peak);

	srv.request.zone = zone;

	if (g_client.call(srv) && l_client.call(srv)) {
		ROS_INFO("Call service success!");
	} else {
		ROS_ERROR("Failed to call service!");
		return 1;
	}

	return 0;
}
