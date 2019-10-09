//*********************************************************
//
// Copyright (c) 2019 Roborock. All rights reserved.
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************


#include "scrubber/Scrubber.h"

namespace rock::scrubber::scrubber {

	Scrubber::Scrubber() : map_id_(-1), zone_id_(-1) {
		ros::NodeHandle n;
		ros::NodeHandle private_nh("~");
		ros::NodeHandle app_nh("roborock_app");
		clean_statistics.total_clean_time = 0;
		clean_statistics.total_clean_area = 0;
		emg_sub_             = n.subscribe("base/emergency_stop", 5, &Scrubber::emergencyStop, this);
		base_vel_sub_        = n.subscribe("base/max_volocity", 5, &Scrubber::baseVel, this);
		state_sub_           = n.subscribe("launcher/scrubber_state", 1, &Scrubber::currentState, this);
		pose_sub_            = n.subscribe("amcl_pose", 1, &Scrubber::currentPose, this);
		map_id_sub_          = n.subscribe("scrubber_database/map_id", 1, &Scrubber::currentMap, this);
		pause_               = n.serviceClient<std_srvs::Empty>("launcher/pause");
		get_scrubber_config_ = n.serviceClient<db_msgs::GetScrubberConfig>("scrubber_database/getScrubberConfig");
		set_clean_config_    = n.serviceClient<scrubber_msgs::SetCleanConfig>("ctlserverset");
		get_clean_config_    = n.serviceClient<scrubber_msgs::GetCleanConfig>("ctlserverget");
		path_config_srv_     = private_nh.advertiseService("setPathConfig", &Scrubber::setPathConfig, this);
		vel_limit_           = private_nh.advertise<geometry_msgs::Twist>("vel_limit", 5);
		app_status_pub_      = app_nh.advertise<roborock_app::RobotStatus>("robotStatus", 5);
		scrubber_thread_.swap(*new std::thread(&Scrubber::exeThread, this));
	}

	void Scrubber::baseVel(const geometry_msgs::Twist::ConstPtr& msg) {
		max_x_base_ = msg->linear.x;
	}

	void Scrubber::emergencyStop(const std_msgs::Bool::ConstPtr& msg) {
		if (msg->data) { //Emergency Stop Event
			std_srvs::Empty empty;
			pause_.call(empty);
		}
	}

	void Scrubber::pubVelocityLimit() {
		geometry_msgs::Twist limit;
		limit.linear.x = max_x_config_ > max_x_base_ ? max_x_base_ : max_x_config_;
		vel_limit_.publish(limit);
	}

	void Scrubber::pubAppStatus() {
		roborock_app::RobotStatus status;
		clean_statistics.total_clean_time = ros::Time::now().sec - clean_start_time_;
		status.stat                       = (int8_t) cur_state_;
		status.power                      = 90; //TODO: Get real battary status
		status.total_clean_area           = (int32_t) clean_statistics.total_clean_area;
		status.total_clean_duration       = clean_statistics.total_clean_time;
		status.clean_water_level          = 50; //TODO:Get real level
		status.dirty_water_level          = 50; //TODO:Get real level
		app_status_pub_.publish(status);
	}

	void Scrubber::exeThread() {
		tf2_ros::Buffer                 tfBuffer;
		tf2_ros::TransformListener      tfListener(tfBuffer);
		geometry_msgs::TransformStamped transformStamped;
		ros::Rate                       r(5);
		while (ros::ok()) {
			if (cur_state_ == States::AUTOMATIC) {
				updateZoneConfig();
				pubVelocityLimit();
			}
			updateCurrentConfig();
			pubAppStatus();
			r.sleep();
		}
	}

	void Scrubber::updateCurrentConfig() {
		scrubber_msgs::GetCleanConfig config;
		if (!get_clean_config_.call(config)) {
			ROS_ERROR_ONCE("Unable to get current config");
			return;
		}

		cur_config_.flow     = config.response.flow;
		cur_config_.vacuum   = config.response.vacuum;
		cur_config_.squeegee = config.response.squeegee;
		cur_config_.brush    = config.response.brush;

		int state = cur_config_.brush + cur_config_.squeegee + cur_config_.vacuum + cur_config_.flow;
		clean_on_ = state > 0;
	}

	bool Scrubber::setPathConfig(scrubber_msgs::SetPathConfigRequest& req, scrubber_msgs::SetPathConfigResponse&) {
		db_msgs::GetScrubberConfig    config;
		scrubber_msgs::SetCleanConfig clean_config;
		config.request.config_id = req.config_id;
		if (get_scrubber_config_.call(config)) {
			clean_config.request.vacuum   = config.response.vacuum;
			clean_config.request.squeegee = config.response.squeegee;
			clean_config.request.flow     = config.response.flow;
			clean_config.request.brush    = config.response.brush;
			return set_clean_config_.call(clean_config);
		}

		return false;
	}

	void Scrubber::currentPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
		if (clean_on_) { //Updating statistics
			double dx = msg->pose.pose.position.x - cur_pose_.pose.pose.position.x;
			double dy = msg->pose.pose.position.y - cur_pose_.pose.pose.position.y;
			double s  = sqrt(dx * dx + dy * dy); //transit distance
			clean_statistics.total_clean_area += s * 0.5; //accumulating area as tiny rectangle, 0.5 is robot diameter
		}

		cur_pose_ = *msg;
	}

	void Scrubber::currentState(const std_msgs::UInt8::ConstPtr& msg) {
		if (cur_state_ == msg->data) {
			return;
		}

		//Update New state
		cur_state_ = msg->data;

		//If transfer to cleaning model
		if (cur_state_ == States::AUTOMATIC || cur_state_ == States::MANUAL) {
			clean_start_time_ = ros::Time::now().sec;//reset statistic
			clean_statistics.total_clean_area = 0;
			clean_statistics.total_clean_time = 0;
		}
	}

	void Scrubber::currentMap(const std_msgs::UInt32::ConstPtr& msg) {
		if (map_id_ == msg->data) return;
		map_id_ = msg->data;

		//-1 means zone or data for the current map were modified, will re publish map id
		// to refresh zones
		if (msg->data == -1) return;

		db_msgs::GetListInMap zones_in_map;
		zones_in_map.request.type        = db_msgs::Type::ZONE;
		zones_in_map.request.by_map_name = false;
		zones_in_map.request.map_id      = map_id_;
		if (!ros::service::call("scrubber_database/getListInMap", zones_in_map)) {
			ROS_ERROR("Unable to get zone info for map %d", map_id_);
			return;
		}

		if (zones_in_map.response.contents.empty()) {
			ROS_INFO("No zone in map %d", map_id_);
			return;
		}

		Zone             zone;
		db_msgs::GetZone get_zone;
		get_zone.request.by_zone_name = false;
		for (db_msgs::Content& i : zones_in_map.response.contents) {
			get_zone.request.zone_id = i.id;
			if (!ros::service::call("scrubber_database/getZone", get_zone)) {
				ROS_ERROR("Fail to get zone %s", i.name.c_str());
				continue;
			}

			zone.zone_id   = i.id;
			zone.type      = get_zone.response.type;
			zone.config_id = get_zone.response.config_id;
			zone.points    = get_zone.response.path;

			zones_.push_back(zone);
		}
	}

	void Scrubber::updateZoneConfig() {
		if (zones_.empty()) return;
		for (Zone zone : zones_) {
			if (inZone(zone)) {
				ROS_INFO("In zone");
				if (zone_id_ == zone.zone_id) return;
				zone_id_ = zone.zone_id;
				scrubber_msgs::SetPathConfig config;
				config.request.config_id = zone.config_id;
				if (!setPathConfig(config.request, config.response)) {
					ROS_ERROR("Set zone config failed");
					zone_id_ = -1; //Try next time
				}

				return;
			}
		}

		if(zone_id_ != -1) { //Moved out from a zone
			zone_id_ = -1;//Not in any zone
		}
	}

	/*
	 * return if current pose in zone
	 */
	bool Scrubber::inZone(Zone& zone) {
		if (zone.type == 1) { //Polygon
			uint16_t size = zone.points.poses.size();
			if (size < 3) {
				ROS_ERROR("Got a polygon zone with less than 3 points");
				return false;
			}

			//Get the which side current pose is to the last line of this polygon
			int8_t side     = sideOf(zone.points.poses.back().pose.position.x,
			                         zone.points.poses.back().pose.position.y,
			                         zone.points.poses.front().pose.position.x,
			                         zone.points.poses.front().pose.position.y);

			//On the line
			if (side == 0) return true;

			// A point is on the interior of this polygons
			// if it is always on the same side of all the line segments
			double        x0, y0, x1, y1;
			for (uint16_t i = 0; i < size - 1; i++) {
				x0 = zone.points.poses[i].pose.position.x;
				y0 = zone.points.poses[i].pose.position.y;
				x1 = zone.points.poses[i + 1].pose.position.x;
				y1 = zone.points.poses[i + 1].pose.position.y;
				if (side != sideOf(x0, y0, x1, y1)) {
					return false;
				}
			}

			return true;
		}

		return false;
	}

	/*
	 * p0(x0, y0), p1(x1, y1) define a path l(p0->p1), current pose is p
	 * Return -1 if p on the right side of l, 0 p on l, 1 p on left side of l
	 */
	uint8_t Scrubber::sideOf(double& x0, double& y0, double& x1, double& y1) {
		double px = cur_pose_.pose.pose.position.x;
		double py = cur_pose_.pose.pose.position.y;

		double side = (py - y0) * (x1 - x0) - (px - x0) * (y1 - y0);
		if (side == 0) return 0;

		return side < 0 ? -1 : 1;
	}
}
