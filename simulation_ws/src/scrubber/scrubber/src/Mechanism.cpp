//*********************************************************
//
// Copyright (c) 2019 Roborock. All rights reserved.
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************


#include "scrubber/Mechanism.h"

namespace rock::scrubber::scrubber {

	Mechanism::Mechanism() : cleaning_(false), emergency_stop_(false) {
		ros::NodeHandle n;
		set_manual_config_ = n.serviceClient<scrubber_msgs::SetManualConfig>("manualset");
		set_clean_config_  = n.serviceClient<scrubber_msgs::SetCleanConfig>("ctlserverset");
		get_clean_config_  = n.serviceClient<scrubber_msgs::GetCleanConfig>("ctlserverget");

		mech_thread_.swap(*new std::thread(&Mechanism::mechRun, this));
	}

	void Mechanism::mechRun() {
		ros::service::waitForService("ctlserverset", ros::Duration(2));
		ros::service::waitForService("ctlserverget", ros::Duration(2));
		ros::Rate r(2);
		while (ros::ok()) {
			update();
			r.sleep();
		}
	}

	void Mechanism::update() {

		scrubber_msgs::GetCleanConfig clean_config;

		if (!get_clean_config_.call(clean_config)) return;

		clean_config_.flow     = clean_config.response.flow;
		clean_config_.brush    = clean_config.response.brush;
		clean_config_.vacuum   = clean_config.response.vacuum;
		clean_config_.squeegee = clean_config.response.squeegee;

		emergency_stop_ = clean_config.response.lock_wheel;
	}

	bool Mechanism::setBrush(int8_t level) {
		if (level < 0) return true; //Unchanged

		scrubber_msgs::SetCleanConfig clean_config;

		clean_config.request.flow = clean_config.request.vacuum
				= clean_config.request.squeegee
				= clean_config.request.lock_wheel = -1;

		if (clean_config_.brush == 0 && level > 0) {        //On
			clean_config.request.brush = 1;
			if (!set_clean_config_.call(clean_config)) {
				ROS_WARN("Failed to lay down brush");
				return false;
			}

			if (!waitBrush(1)) {
				ROS_WARN("Brush control timeout");
				return false;
			}

			clean_config.request.brush = level;
			if (level != 1 && !set_clean_config_.call(clean_config)) {
				ROS_WARN("Failed to start brush");
				return false;
			}
			ros::Duration(0.2).sleep();
		} else if (clean_config_.brush > 0 && level == 0) { //OFF
			clean_config.request.brush = 1;
			if (clean_config_.brush > 1 && !set_clean_config_.call(clean_config)) {
				ROS_WARN("Failed to stop brush");
				return false;
			}

			ros::Duration(0.2).sleep();
			clean_config.request.brush = level;
			if (!set_clean_config_.call(clean_config)) {
				ROS_WARN("Failed to lift brush");
				return false;
			}

			if (!waitBrush(level)) {
				ROS_WARN("Brush control timeout");
				return false;
			}
		} else {                                            //Speed change
			clean_config.request.brush = level;
			if (!set_clean_config_.call(clean_config)) {
				ROS_WARN("Failed to reset brush");
				return false;
			}
		}

		return true;
	}

	bool Mechanism::setVacuum(int8_t level) {
		if (level < 0) return true;

		scrubber_msgs::SetCleanConfig clean_config;

		clean_config.request.flow = clean_config.request.brush
				= clean_config.request.squeegee
				= clean_config.request.lock_wheel = -1;

		clean_config.request.vacuum = level;

		if (!set_clean_config_.call(clean_config)) {
			ROS_WARN("Failed to set vacuum");
			return false;
		}

		return true;
	}

	bool Mechanism::setSqueegee(int8_t level) {
		if (level < 0) return true;

		scrubber_msgs::SetCleanConfig clean_config;

		clean_config.request.flow = clean_config.request.brush
				= clean_config.request.vacuum
				= clean_config.request.lock_wheel = -1;

		clean_config.request.squeegee = level > 0 ? 1 : 0;

		if (!set_clean_config_.call(clean_config)) {
			ROS_WARN("Failed to set squeegee");
			return false;
		}

		if (!waitSqueegee(clean_config.request.squeegee)) {
			ROS_WARN("Squeegee control time out");
			return false;
		}

		return true;
	}

	bool Mechanism::setFlow(int8_t level) {
		if (level < 0) return true;

		scrubber_msgs::SetCleanConfig clean_config;

		clean_config.request.brush = clean_config.request.squeegee
				= clean_config.request.vacuum
				= clean_config.request.lock_wheel = -1;

		clean_config.request.flow = level;

		if (!set_clean_config_.call(clean_config)) {
			ROS_WARN("Failed to set flow");
			return false;
		}
		ros::Duration(0.5).sleep();
		return true;
	}

	bool Mechanism::setCleanConfig(Config& target_config) {
		bool flow, brush, vacuum, squeegee;
		flow     = setFlow(target_config.flow);
		brush    = setBrush(target_config.brush);
		vacuum   = setVacuum(target_config.vacuum);
		squeegee = setSqueegee(target_config.squeegee);
		return flow && brush && vacuum && squeegee;
	}

	bool Mechanism::setManualConfig(bool enable, uint8_t direction, float speed) {
		scrubber_msgs::SetManualConfig manual_config;
		manual_config.request.enable    = enable ? 1 : 0;
		manual_config.request.direction = direction ? 1 : 0;
		manual_config.request.speed     = speed;
		if (!set_manual_config_.call(manual_config)) {
			ROS_WARN("Failed to set manual configs");
			return false;
		}

		manual_config_.enable    = enable;
		manual_config_.direction = direction;
		manual_config_.speed     = speed;
		return true;
	}

	bool Mechanism::pause() {
		Config config;
		config.brush    = clean_config_.brush > 0 ? 1 : -1;
		config.flow     = clean_config_.flow > 0 ? 0 : -1;
		config.vacuum   = clean_config_.vacuum > 0 ? 0 : -1;
		config.squeegee = -1;
		return setCleanConfig(config);
	}

	bool Mechanism::waitBrush(int8_t target_state) {
		float      timeout = 8;
		for (float t       = 0; t < timeout;) {
			ros::Duration(0.5).sleep();
			if (clean_config_.brush == target_state) {
				return true;
			}
			t += 0.5;
		}
		return false;
	}

	bool Mechanism::waitSqueegee(int8_t target_state) {
		float      timeout = 8;
		for (float t       = 0; t < timeout;) {
			ros::Duration(0.5).sleep();
			if (clean_config_.squeegee == target_state) {
				return true;
			}

			t += 0.5;
		}

		return false;
	}
}
