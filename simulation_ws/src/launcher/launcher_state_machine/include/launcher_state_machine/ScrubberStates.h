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
#include <std_msgs/UInt32.h>
#include <std_msgs/Int32.h>
#include <launcher_msgs/LauncherAction.h>
#include <launcher_msgs/State.h>
#include <scrubber_msgs/UpdateStatistics.h>
#include <scrubber_msgs/SetManualConfig.h>
#include <atomic>

namespace rock::scrubber::launcher {
	typedef launcher_msgs::State States;

	class ScrubberStates {
	public:
		ScrubberStates();

		void shiftState(const uint8_t&);

		void mapCallback(const std_msgs::UInt32::ConstPtr& msg);

		void setRelocated(bool relocated) { relocated_ = relocated; }

		void setError(uint8_t error_code) { error_ = error_code; }

		void setConfig(uint32_t config_code) { config_ = config_code; }

		bool mapAvailable() const { return map_available_; }

		bool relocated() const { return relocated_; }

		bool gripControl(bool enable = true);

		uint8_t getError() const { return error_; }

		uint8_t getConfig() const { return config_; }

		uint8_t getState() const { return state_; }

		uint32_t getMapId() const { return map_id_; }

		uint8_t getLastState() const { return last_state_; }

		std::string stateName(const uint8_t& state);

	private:
		ros::Subscriber map_sub_;
		ros::ServiceClient manual_config_;

		std::map<uint8_t, std::string> name_;

		std::atomic<uint8_t>  state_;
		std::atomic<uint8_t>  last_state_;
		std::atomic<uint8_t>  error_;
		std::atomic<uint32_t> config_;
		std::atomic<uint32_t> map_id_;
		std::atomic<bool>     charging_;
		std::atomic<bool>     relocated_;
		std::atomic<bool>     map_available_;
	};
}

