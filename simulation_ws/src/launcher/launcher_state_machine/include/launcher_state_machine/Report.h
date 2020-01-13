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
#include <vector>
#include <cstdint>
#include <cmath>
#include <db_msgs/AddCleanRecord.h>

namespace rock::scrubber::launcher {
	class SimpleMap {
	public:
		//Map bottom left corner
		double min_x, min_y;

		//Map scale (m/cell)
		double scale;

		//Map demensions
		int size_x, size_y;

		//Map data
		std::vector<int8_t> cells;

		//World coords to map cell
		int cellX(double world_x) {
			return std::floor((world_x - min_x) / scale + 0.5);
		}

		int cellY(double world_y) {
			return std::floor((world_y - min_y) / scale + 0.5);
		}

		bool validCell(int cell_x, int cell_y){
			return cell_x >= 0 && cell_y >= 0 && cell_x < size_x && cell_y < size_y;
		}

		int index(int cell_x, int cell_y){
			return cell_x + cell_y * size_x;
		}
	};

	class Report {
	public:
		Report();

		Report(double min_x, double min_y, double max_x, double max_y, double resolution);

		void paint(double world_x, double world_y);

		void start(){
			start_time = ros::Time::now().sec;
		}

		void reset(){
			clean_time = 0;
			clean_area = 0;
			last_clean_area = 0;
			last_clean_time = 0;
		}

		void pause(){
			clean_time += ros::Time::now().sec - start_time;
		}

		bool saveReport(uint32_t plan_id, uint32_t path_id, uint32_t map_id, uint8_t config_id);

		uint32_t getCleanArea() const{
			if(clean_map.cells.empty()) return 0;
			int cell_count = 0;
			for(auto& cell: clean_map.cells){
				cell_count += cell;
			}
			//Number of painted cells * cell_area, unit square cm
			return std::floor(cell_count * clean_map.scale * clean_map.scale * 10000);
		}

		uint32_t getCleanTime() const{
			return clean_time + (ros::Time::now().sec - start_time);
		}

		uint32_t getTimeIncre(){
			uint32_t dt = getCleanTime() - last_clean_time;
			last_clean_time = getCleanTime();
			return dt;
		}

		uint32_t getAreaIncre(){
			uint32_t dt = getCleanArea() - last_clean_area;
			last_clean_area = getCleanArea();
			return dt;
		}
	private:
		SimpleMap  clean_map;
		uint32_t   clean_area; //square cm
		uint32_t   clean_time;
		uint32_t   start_time;
		uint32_t   last_clean_time;
		uint32_t   last_clean_area;
	};
}

