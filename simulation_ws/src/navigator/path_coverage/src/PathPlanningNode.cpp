#include <boost/shared_ptr.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <fstream>
#include <iostream>
#include <std_srvs/Empty.h>

#include "CleaningPathPlanner.h"
#include "path_coverage/GetPathInZone.h"

#define RECORD_IN_FILE 0

namespace cm = costmap_2d;
namespace gm = geometry_msgs;

using cm::Costmap2D;
using cm::Costmap2DROS;
using gm::PoseStamped;
using std::string;
using std::vector;

class PathPlanning {
public:
	PathPlanning();
	~PathPlanning();

private:
	bool getPathInZone(path_coverage::GetPathInZone::Request& req, path_coverage::GetPathInZone::Response& resp);

	ros::NodeHandle nh_;
	ros::Publisher pub_;
	ros::ServiceServer get_path_in_zone_srv_;
};

boost::shared_ptr<PathPlanning> path_planning_node_ptr;

int main(int argc, char** argv) {
	ros::init(argc, argv, "path_planning_node");
	path_planning_node_ptr.reset(new PathPlanning());

	ros::spin();
	ros::shutdown();
	return 0;
}

PathPlanning::PathPlanning() {
	get_path_in_zone_srv_ = nh_.advertiseService("getPathInZone", &PathPlanning::getPathInZone, this);
	pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 2);
}

PathPlanning::~PathPlanning() {
}

bool PathPlanning::getPathInZone(path_coverage::GetPathInZone::Request& req, path_coverage::GetPathInZone::Response& resp) {
	cout << "getPathInZone: receive call request." << endl;
	for (auto i = 0; i < req.zone.poses.size(); i++) {
		cout << "peck " << i + 1 << ": ["
			 << req.zone.poses[i].pose.position.x << ", "
			 << req.zone.poses[i].pose.position.y << "]" << endl;
	}

	geometry_msgs::PoseWithCovarianceStamped fake_pose;
	fake_pose.header.stamp = ros::Time::now();
	fake_pose.header.frame_id = "map";
	fake_pose.pose.pose.position.x = 1000.0;
	fake_pose.pose.pose.position.y = 1000.0;
	fake_pose.pose.pose.position.z = 0.0;
	fake_pose.pose.pose.orientation.x = 0.0;
	fake_pose.pose.pose.orientation.y = 0.0;
	fake_pose.pose.pose.orientation.z = 0.0;
	fake_pose.pose.pose.orientation.w = 1.0;
	fake_pose.pose.covariance[6*0+0] = 0.5 * 0.5;
	fake_pose.pose.covariance[6*1+1] = 0.5 * 0.5;
	fake_pose.pose.covariance[6*5+5] = PI/12.0 * PI/12.0;
	pub_.publish(fake_pose);
	sleep(1);

	if(!ros::service::waitForService("move_base_flex/clear_costmaps", ros::Duration(5))) {
		ROS_ERROR("move_base_flex/clear_costmaps not ready!");
	} else {
		ros::ServiceClient clearClient = nh_.serviceClient<std_srvs::Empty>("move_base_flex/clear_costmaps");
		std_srvs::Empty srv;
		if (clearClient.call(srv))
			ROS_INFO("Clear costmap success!");
		else
			ROS_ERROR("Clear costmap fail!");
	}

	tf2_ros::Buffer            buffer(ros::Duration(1));
	tf2_ros::TransformListener tf(buffer);

	costmap_2d::Costmap2DROS costmap("cleaning_costmap", buffer);
	//planner_costmap_ros_->pause();

	boost::shared_ptr<CleaningPathPlanning> cleaning_ptr;
	cleaning_ptr.reset(new CleaningPathPlanning(&costmap, req.zone));

	resp.paths = cleaning_ptr->GetPathInROS();

	fake_pose.header.stamp = ros::Time::now();
	fake_pose.pose.pose.position.x = 0.0;
	fake_pose.pose.pose.position.y = 0.0;
	fake_pose.pose.pose.orientation.w = 1.0;
	pub_.publish(fake_pose);

	if (resp.paths.empty())
		return false;

#if RECORD_IN_FILE
	std::ofstream f("/home/rock-trl/workspace/scrubber_test/navigator/pose_array.txt", ios::app | ios::out);
	for (auto& path: resp.paths) {
		for (auto& pose: path.poses) {
			tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y,
			                 pose.pose.orientation.z, pose.pose.orientation.w);
			tf::Matrix3x3 m(q);
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);
			f << pose.pose.position.x << " " << pose.pose.position.y << " " << yaw << std::endl;
		}
		f << "EOP" << std::endl;
	}

	f.close();
#endif

	return true;
}
