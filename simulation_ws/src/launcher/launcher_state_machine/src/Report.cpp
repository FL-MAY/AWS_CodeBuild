//*********************************************************
//
// Copyright (c) 2019 Roborock. All rights reserved.
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************

#include "launcher_state_machine/Report.h"

namespace rock::scrubber::launcher {

	Report::Report() : clean_area(0), clean_time(0), start_time(0) {
		reset();
	}

	Report::Report(double min_x, double min_y, double max_x, double max_y, double resolution) :
			clean_area(0), clean_time(0), start_time(0) {
		clean_map.min_x  = min_x;
		clean_map.min_y  = min_y;
		clean_map.scale  = resolution;
		clean_map.size_x = std::floor((max_x - min_x) / clean_map.scale + 0.5);
		clean_map.size_y = std::floor((max_y - min_y) / clean_map.scale + 0.5);
		clean_map.cells  = std::vector<int8_t>(clean_map.size_x * clean_map.size_y, 0);
		ROS_INFO("Clean map min_x: %.2f min_y: %.2f resolution: %.2f size_x: %d size_y: %d",
		         min_x, min_y, resolution, clean_map.size_x, clean_map.size_y);
		reset();
		start();
	}

	void Report::paint(double world_x, double world_y) {
		//Mark a 0.8m * 0.8m square
		int      x_min = clean_map.cellX(world_x - 0.4);
		int      x_max = clean_map.cellX(world_x + 0.4);
		int      y_min = clean_map.cellY(world_y - 0.4);
		int      y_max = clean_map.cellY(world_y + 0.4);
		for (int i     = x_min; i < x_max; i++) {
			for (int j = y_min; j < y_max; j++) {
				if (clean_map.validCell(i, j)) {
					clean_map.cells[clean_map.index(i, j)] = 1;
				}
			}
		}
	}

	bool Report::saveReport(uint32_t plan_id, uint32_t path_id, uint32_t map_id, uint8_t config_id) {
		db_msgs::AddCleanRecord record;
		record.request.record.path_id           = path_id;
		record.request.record.plan_id           = plan_id;
		record.request.record.map_id            = map_id;
		record.request.record.config_id         = config_id;
		record.request.record.duration          = getCleanTime();
		record.request.record.area              = getCleanArea();
		record.request.record.cover_rate        = 100*float(record.request.record.area) / (clean_map.cells.size() * 25);
		//TODO:Water & Power comsumption
		record.request.record.water_comsumption = 5;
		record.request.record.power_comsumption = 10;
		if(!ros::service::call("scrubber_database/addCleanRecord", record)){
			ROS_WARN("Failed to record current report plan id %d", plan_id);
			return false;
		}

		return true;
	}
}