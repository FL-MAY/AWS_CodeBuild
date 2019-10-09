//*********************************************************
//
// Copyright (c) 2019 Roborock. All rights reserved.
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************

#pragma once

#include <thread>
#include <vector>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <db_msgs/GetScrubberConfig.h>
#include <db_msgs/GetListInMap.h>
#include <db_msgs/GetZone.h>
#include <db_msgs/GetPath.h>
#include <db_msgs/Content.h>
#include <db_msgs/Type.h>
#include <launcher_msgs/MissionResult.h>
#include <scrubber_msgs/SetCleanConfig.h>
#include <scrubber_msgs/GetCleanConfig.h>
#include <scrubber_msgs/SetPathConfig.h>
#include <roborock_app/RobotStatus.h>
#include <roborock_app/GetCleanRegions.h>
#include <roborock_app/GetCleanSubRegions.h>

namespace rock::scrubber::scrubber {
	typedef launcher_msgs::MissionResult States;

	struct Zone {
		uint32_t       zone_id {};
		uint8_t        type {};
		uint32_t       config_id {};
		nav_msgs::Path points;
	};

	struct Config {
		uint8_t brush;
		uint8_t squeegee;
		uint8_t flow;
		uint8_t vacuum;
	};

	struct CleanStatistics {
		int32_t total_clean_time;
		double  total_clean_area;
	};

	class Scrubber {
	public:
		Scrubber();

		~Scrubber() = default;

	private:
		void emergencyStop(const std_msgs::Bool::ConstPtr& msg);

		void baseVel(const geometry_msgs::Twist::ConstPtr& msg);

		void currentState(const std_msgs::UInt8::ConstPtr& msg);

		void currentPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

		void currentMap(const std_msgs::UInt32::ConstPtr& msg);

		bool setPathConfig(scrubber_msgs::SetPathConfigRequest& req, scrubber_msgs::SetPathConfigResponse&);

		void updateZoneConfig();

		void updateCurrentConfig();

		bool inZone(Zone& zone);

		uint8_t sideOf(double& x0, double& y0, double& x1, double& y1);

		void pubVelocityLimit();

		void pubAppStatus();

		void exeThread();

		uint8_t           cur_state_;
		int32_t           map_id_;
		int32_t           zone_id_;
		int32_t           clean_start_time_;//sec
		double            max_x_config_;
		double            max_x_base_;
		bool              clean_on_;
		Config            cur_config_;
		CleanStatistics   clean_statistics;
		std::vector<Zone> zones_;

		ros::Publisher                           vel_limit_;
		ros::Publisher                           app_status_pub_;
		ros::Subscriber                          state_sub_;
		ros::Subscriber                          emg_sub_;
		ros::Subscriber                          base_vel_sub_;
		ros::Subscriber                          pose_sub_;
		ros::Subscriber                          map_id_sub_;
		ros::ServiceServer                       path_config_srv_;
		ros::ServiceClient                       pause_;
		ros::ServiceClient                       get_scrubber_config_;
		ros::ServiceClient                       set_clean_config_;
		ros::ServiceClient                       get_clean_config_;
		std::thread                              scrubber_thread_;
		geometry_msgs::PoseWithCovarianceStamped cur_pose_;
	};
}

