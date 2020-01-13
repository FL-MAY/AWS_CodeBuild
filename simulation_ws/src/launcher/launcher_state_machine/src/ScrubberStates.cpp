//*********************************************************
//
// Copyright (c) 2019 Roborock. All rights reserved.
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************

#include "launcher_state_machine/ScrubberStates.h"

namespace rock::scrubber::launcher {
	ScrubberStates::ScrubberStates()
			: charging_(false), relocated_(false), map_available_(false), error_(0), config_(0), state_(States::IDLE),
			  last_state_(States::IDLE) {

		name_[States::IDLE]            = "IDLE";
		name_[States::AUTOMATIC]       = "Automatic";
		name_[States::AUTOMATIC_CLEAN] = "Automatic Clean";
		name_[States::AUTOMATIC_GOTO]  = "Automatic Goto";
		name_[States::CHARGING]        = "Charging";
		name_[States::UPDATING]        = "Updating";
		name_[States::MANUAL]          = "Manual";
		name_[States::MAPPING]         = "Mapping";
		name_[States::TRACKING]        = "Tracking";
		name_[States::PAUSE]           = "Pause";
		name_[States::PREPARING]       = "Preparing";
		ros::NodeHandle n;
		map_sub_ = n.subscribe("scrubber_database/map_id", 1, &ScrubberStates::mapCallback, this);
		if(!ros::service::waitForService("manualset", ros::Duration(5))){
			ROS_ERROR("Couldn't get grip control, it may not work as expected");
		}

		manual_config_ = n.serviceClient<scrubber_msgs::SetManualConfig>("manualset");
	}

	void ScrubberStates::shiftState(const uint8_t& newState) {
		uint8_t curState = state_;
		last_state_ = curState;
		state_      = newState;
		ROS_DEBUG("%s --> %s", stateName(last_state_).c_str(), stateName(state_).c_str());
		ROS_INFO("%s --> %s", stateName(last_state_).c_str(), stateName(state_).c_str());
	}

	std::string ScrubberStates::stateName(const uint8_t& state) {
		if (name_.find(state) != name_.end()) {
			return name_[state];
		}

		return "?";
	}

	void ScrubberStates::mapCallback(const std_msgs::UInt32::ConstPtr& msg) {
		map_id_        = msg->data;
		map_available_ = true;
	}

	bool ScrubberStates::gripControl(bool enable) {
		std::string state = enable?"Enable" : "Disable";
		ROS_INFO("%s grip control", state.c_str());
		//Default direction forward
		scrubber_msgs::SetManualConfig config;
		config.request.enable = enable;
		config.request.speed  = 0.5;
		return manual_config_.call(config);
	}
}