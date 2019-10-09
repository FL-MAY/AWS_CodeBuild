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

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <thread>
#include "Mission.h"
#include "ScrubberStates.h"

namespace rock::scrubber::launcher {
	class Tracking : public Mission {
	public:
		explicit Tracking(std::shared_ptr<ScrubberStates>& states);

		~Tracking() override;

		void start() override;

		void pause() override;

		void resume() override;

		void cancel() override;

	private:
		void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

		void tracking();

		bool active_,ready_;

		std::shared_ptr<ScrubberStates> states_;

		ros::Subscriber pose_sub_;
		ros::Publisher  path_pub_;
		nav_msgs::Path  path_;
		std::thread     tracking_thread_;
	};
}

