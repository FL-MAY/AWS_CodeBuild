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
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <amcl/RectPara.h>
#include <path_coverage/GetPathInZone.h>
#include <launcher_msgs/LauncherAction.h>
#include <launcher_msgs/PrepareNode.h>
#include <launcher_msgs/State.h>
#include <scrubber_msgs/SetCleanConfig.h>
#include <scrubber_msgs/GetCleanConfig.h>
#include <scrubber_msgs/SetPathConfig.h>
#include <scrubber_msgs/UpdateStatistics.h>
#include <scrubber_msgs/SetManualConfig.h>
#include <zlib.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <db_msgs/Task.h>
#include <db_msgs/Delete.h>
#include <db_msgs/Rename.h>
#include <db_msgs/AddPlan.h>
#include <db_msgs/AddMap.h>
#include <db_msgs/AddPath.h>
#include <db_msgs/AddZone.h>
#include <db_msgs/GetScrubberConfig.h>
#include <db_msgs/GetListInMap.h>
#include <db_msgs/GetList.h>
#include <db_msgs/GetMap.h>
#include <db_msgs/GetZone.h>
#include <db_msgs/GetPath.h>
#include <db_msgs/GetPlan.h>
#include <db_msgs/GetTask.h>
#include <db_msgs/UpdateInit.h>
#include <db_msgs/UpdatePlan.h>
#include <db_msgs/UpdateMap.h>
#include <db_msgs/SetMarkPoint.h>
#include <db_msgs/GetMarkPoint.h>
#include <db_msgs/GetConfigScale.h>
#include <db_msgs/GetConfigMode.h>
#include <db_msgs/SetConfigMode.h>
#include <db_msgs/BindAutofill.h>
#include <db_msgs/GetAutofillZone.h>
#include <db_msgs/GetCleanRecords.h>
#include <db_msgs/GetRecordSummary.h>
#include <db_msgs/GetRecordInfo.h>
#include <db_msgs/GetPathsInfo.h>
#include <db_msgs/Content.h>
#include <db_msgs/TypePath.h>
#include <db_msgs/TypeQuery.h>
#include <db_msgs/TypeZone.h>
#include <db_msgs/TypeConfig.h>
#include <db_msgs/TypePlan.h>
#include <db_msgs/ResetMap.h>
#include <db_msgs/ConfigMode.h>
#include <db_msgs/ConfigValueID.h>
#include <roborock_app/RobotStatus.h>
#include <roborock_app/RobotErrCode.h>
#include <roborock_app/CmdErrCode.h>
#include <roborock_app/StatForAppCode.h>
#include <roborock_app/GetCleanRegions.h>
#include <roborock_app/GetCleanSubRegions.h>
#include <roborock_app/GetRegionMap.h>
#include <roborock_app/GetRegionPaths.h>
#include <roborock_app/GetPaths.h>
#include <roborock_app/RenameRegion.h>
#include <roborock_app/DeleteRegion.h>
#include <roborock_app/StartAutoTask.h>
#include <roborock_app/StopAutoTask.h>
#include <roborock_app/ExitAutoTask.h>
#include <roborock_app/StartPathRecord.h>
#include <roborock_app/SavePath.h>
#include <roborock_app/ExitPathRecord.h>
#include <roborock_app/EnterMapNew.h>
#include <roborock_app/ExitMapNew.h>
#include <roborock_app/SaveMapNew.h>
#include <roborock_app/EnterMapEdit.h>
#include <roborock_app/ExitMapEdit.h>
#include <roborock_app/SaveMapEdit.h>
#include <roborock_app/Locate.h>
#include <roborock_app/LocateRect.h>
#include <roborock_app/GetCleanConfig.h>
#include <roborock_app/MapExtendAction.h>
#include <roborock_app/ResetMap.h>
#include <roborock_app/SubRegionCompose.h>
#include <roborock_app/GetCleanRecord.h>
#include <roborock_app/GetCleanRecordMap.h>
#include <roborock_app/GetCleanRecordSummary.h>
#include <roborock_app/SwitchTask.h>
#include <roborock_app/PointFix.h>
#include <roborock_app/PathType.h>
#include <roborock_app/MarkPointType.h>
#include <roborock_app/ManualConfig.h>
#include <roborock_app/Speed.h>
#include <roborock_app/UpDown.h>
#include <roborock_app/Direction.h>
#include <roborock_app/ConfigIndex.h>
#include <roborock_app/ConfigType.h>
#include <roborock_app/PointFixType.h>
#include <roborock_app/CleanConfig.h>
#include <roborock_app/SubRegionType.h>


namespace rock::scrubber::scrubber {

	struct MapInfo {
		// Map origin; the map is a viewport onto a conceptual larger map.
		double origin_x, origin_y;

		// Map scale (m/cell)
		double scale;

		// Map dimensions (cells)
		int size_x, size_y;

		int worldXtoGridX(double x) {
			return floor((x - origin_x) / scale + 0.5);//
		}

		int worldYtoGridY(double y) {
			return floor((y - origin_y) / scale + 0.5);//
		}

	};

	struct Zone {
		uint32_t                          zone_id {};
		uint8_t                           type {};
		uint32_t                          config_id {};
		std::vector<geometry_msgs::Point> points;
	};

	struct Manual {
		bool    enable;
		uint8_t direction;
		float   speed;
	};

	struct Config {
		int8_t brush;
		int8_t squeegee;
		int8_t flow;
		int8_t vacuum;
	};

	struct ConfigScale {
		uint8_t low;
		uint8_t medium;
		uint8_t high;
	};

	struct CleanStatistics {
		int32_t total_clean_time;
		int32_t total_clean_area;
	};

	struct Battery {
		uint8_t soc;
		uint8_t charging_state;
	};

	struct FluidLevel {
		uint8_t clean_water;
		uint8_t dirty_water;
	};

    /**
	* Given three colinear points p, q, r, the function checks if
	* point q lies on line segment 'pr'
	*/
    bool onSegment(geometry_msgs::Point p, geometry_msgs::Point q, geometry_msgs::Point r);

	/**
	 *To find orientation of ordered triplet (p, q, r).
     *The function returns following values
     *0 --> p, q and r are colinear
     *1 --> Clockwise
     *2 --> Counterclockwise
	*/
	int orientation(geometry_msgs::Point p, geometry_msgs::Point q, geometry_msgs::Point r);

	/**
	 * The main function that returns true if line segment 'p1q1'
	 * and 'p2q2' intersect.
	 */
	bool intersect(geometry_msgs::Point p1, geometry_msgs::Point q1,
	                 geometry_msgs::Point p2, geometry_msgs::Point q2);

	nav_msgs::Path pointsToPath(std::vector<geometry_msgs::Point>& points);

	bool rectEqual(const roborock_app::Rect& rec1,const roborock_app::Rect& rec2);

	std::vector<geometry_msgs::Point> rectToPointArray(const roborock_app::Rect& rect);

	/**
	 * p0(x0, y0), p1(x1, y1) define a path l(p0->p1), candidate pose is px,py
	 * Return -1 if p on the right side of l, 0 p on l, 1 p on left side of l
	 */
	uint8_t sideOf(double& x0, double& y0, double& x1, double& y1,double& px, double& py);

	//Type converter
	uint8_t TypeZoneToappMkpType(uint8_t& type_zone);

	uint8_t appMkpTypeToTypeZone(uint8_t& app_type);

	int8_t appSpeedTodbSpeed(uint8_t& app_speed, bool vel = false);

	int8_t dbSpeedToAppSpeed(uint8_t& db_speed);

	int8_t cfgModeToappCfgIndex(uint8_t cfgMode);

	int8_t appCfgIndexToCfgMode(uint8_t appConfigIndex);

	nav_msgs::Path rectToPath(roborock_app::Rect& rect);

	roborock_app::Rect pathToRect(nav_msgs::Path& path);

	//Geometry related
	void diagonal(std::vector<geometry_msgs::Point>& zone,
	              geometry_msgs::Point& min, geometry_msgs::Point& max);

	bool inZone(std::vector<geometry_msgs::Point>& zone, geometry_msgs::Point& point);

	bool overlap(std::vector<geometry_msgs::Point>& zone_one, std::vector<geometry_msgs::Point>& zone_two);

	std::vector<geometry_msgs::Point> crossRoute(std::vector<geometry_msgs::Point>& zone);

	double distance(geometry_msgs::Point p1, geometry_msgs::Point p2);
	/**
	 * Compress data
	 * @param src source data
	 * @param srcLen source data size
	 * @param dest destination memery
	 * @param destLen memery size
	 * @return
	 */
	int gzCompress(const char* src, int srcLen, char* dest, int destLen);

	int gzDecompress(const char* src, int srcLen, const char* dst, int dstLen);

	bool compressMap(nav_msgs::OccupancyGrid& raw_map, nav_msgs::OccupancyGrid& compressed_map);
}