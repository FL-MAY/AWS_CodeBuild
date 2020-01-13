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
#include "scrubber/scrubber_util.h"

namespace rock::scrubber::scrubber {
	class Mechanism {
	public:
		Mechanism();

		~Mechanism() = default;

		bool setManualConfig(bool enable = true, uint8_t direction = 0, float speed = 0.5);

		bool setCleanConfig(Config& target_config);

		bool setBrush(int8_t level);

		bool setVacuum(int8_t level);

		bool setSqueegee(int8_t level);

		bool setFlow(int8_t level);

		bool pause();

		bool isEmergency() const {
			return emergency_stop_;
		}

		const Config& getCleanConfig() {
			return clean_config_;
		}

		const Manual& getManualConfig() {
			return manual_config_;
		}

	private:
		void mechRun();

		void update();

		bool waitBrush(int8_t target_state);

		bool waitSqueegee(int8_t target_state);

		bool cleaning_;

		bool emergency_stop_;

		std::thread mech_thread_;

		ros::ServiceClient set_manual_config_;
		ros::ServiceClient set_clean_config_;
		ros::ServiceClient get_clean_config_;

		Config clean_config_;
		Manual manual_config_;
	};
}

