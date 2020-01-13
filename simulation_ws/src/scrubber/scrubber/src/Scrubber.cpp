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

	Scrubber::Scrubber() : map_id_(-1), zone_id_(-1), tf2_(tf_buffer_), app_pub_map_(false), goal_active_(false),
	                       clean_mech_(new Mechanism()) {
		ros::NodeHandle n;
		ros::NodeHandle private_nh("~");
		clean_statistics.total_clean_time = 0;
		clean_statistics.total_clean_area = 0;
		app_status_.stat_for_app          = roborock_app::StatForAppCode::CLEAN_STOPPED;
		app_status_.brush_disc_stat       = roborock_app::UpDown::UP;
		app_status_.water_suck_stat       = roborock_app::UpDown::UP;
		app_status_.walk_direction        = roborock_app::Direction::FORWARD;
		launcher_              = new actionlib::SimpleActionClient<launcher_msgs::LauncherAction>("LauncherServer",
		                                                                                          true);
		emg_sub_               = n.subscribe("base/emergency_stop", 5, &Scrubber::emergencyStop, this);
		base_vel_sub_          = n.subscribe("base/max_volocity", 5, &Scrubber::baseVel, this);
		state_sub_             = n.subscribe("launcher/scrubber_state", 1, &Scrubber::currentState, this);
		pose_sub_              = n.subscribe("amcl_pose", 1, &Scrubber::currentPose, this);
		map_id_sub_            = n.subscribe("scrubber_database/map_id", 1, &Scrubber::currentMap, this);
		map_data_sub_          = n.subscribe("map", 1, &Scrubber::currentMapData, this);
		ultrasonic_sub_        = n.subscribe("ultrasonic_meas_info", 1, &Scrubber::ultrasonic, this);
		path_sub_              = n.subscribe("launcher/tracking_path", 1, &Scrubber::trackingCallback, this);
		scan_sub_              = n.subscribe("scan", 1, &Scrubber::laserScan, this);
		battery_sub_           = n.subscribe("bms_charge", 1, &Scrubber::batteryStatus, this);
		fluid_sub_             = n.subscribe("levelmeasurement", 1, &Scrubber::fluidLevel, this);
		pause_                 = n.serviceClient<std_srvs::Empty>("launcher/pause");
		update_statistics_srv_ = private_nh.advertiseService("updateStatistics", &Scrubber::updateStatistics, this);
		cross_bump_srv_        = private_nh.advertiseService("crossBump", &Scrubber::crossBump, this);
		//Database related
		ros::NodeHandle db_nh("scrubber_database");
		db_get_map_           = db_nh.serviceClient<db_msgs::GetMap>("getMap");
		get_scrubber_config_  = db_nh.serviceClient<db_msgs::GetScrubberConfig>("getScrubberConfig");
		db_add_plan_          = db_nh.serviceClient<db_msgs::AddPlan>("addPlan");
		db_add_path_          = db_nh.serviceClient<db_msgs::AddPath>("addPath");
		db_add_zone_          = db_nh.serviceClient<db_msgs::AddZone>("addZone");
		db_get_path_          = db_nh.serviceClient<db_msgs::GetPath>("getPath");
		db_paths_info_        = db_nh.serviceClient<db_msgs::GetPathsInfo>("getPathsInfo");
		db_get_plan_          = db_nh.serviceClient<db_msgs::GetPlan>("getPlan");
		db_get_zone_          = db_nh.serviceClient<db_msgs::GetZone>("getZone");
		db_get_list_          = db_nh.serviceClient<db_msgs::GetList>("getList");
		db_get_list_in_map_   = db_nh.serviceClient<db_msgs::GetListInMap>("getListInMap");
		db_get_autofill_zone_ = db_nh.serviceClient<db_msgs::GetAutofillZone>("getAutofillZone");
		db_get_config_scale_  = db_nh.serviceClient<db_msgs::GetConfigScale>("getConfigScale");
		db_delete_            = db_nh.serviceClient<db_msgs::Delete>("delete");
		db_update_plan_       = db_nh.serviceClient<db_msgs::UpdatePlan>("updatePlan");
		db_update_plan_init_  = db_nh.serviceClient<db_msgs::UpdateInit>("updatePlanInit");
		db_update_map_        = db_nh.serviceClient<db_msgs::UpdateMap>("updateMap");
		//Hardware related
		ultrasonic_pub_       = n.advertise<sensor_msgs::Range>("ultrasonic", 5);
		path_config_srv_      = private_nh.advertiseService("setPathConfig", &Scrubber::setPathConfig, this);
		vel_limit_            = private_nh.advertise<geometry_msgs::Twist>("vel_limit", 5);
		//App related
		ros::NodeHandle app_nh("roborock_scrubber");
		app_status_pub_        = app_nh.advertise<roborock_app::RobotStatus>("app_status_topic", 5);
		app_map_info_pub_      = app_nh.advertise<roborock_app::RosMapInfo>("app_map_topic", 1);
		app_manual_config_     = app_nh.advertiseService("app_manual_config", &Scrubber::appManualConfig, this);
		app_get_map_list_      = app_nh.advertiseService("app_get_clean_regions", &Scrubber::appGetMapList, this);
		app_get_plan_list_     = app_nh.advertiseService("app_get_clean_sub_regions", &Scrubber::appGetPlanList, this);
		app_get_map_           = app_nh.advertiseService("app_get_region_map", &Scrubber::appGetMap, this);
		app_get_paths_         = app_nh.advertiseService("app_get_region_paths", &Scrubber::appGetPaths, this);
		app_get_plan_paths_    = app_nh.advertiseService("app_load_paths", &Scrubber::appGetPlanPaths, this);
		app_start_auto_task_   = app_nh.advertiseService("app_start_auto_task", &Scrubber::appStartAutoTask, this);
		app_stop_auto_task_    = app_nh.advertiseService("app_stop_auto_task", &Scrubber::appStopAutoTask, this);
		app_switch_auto_task_  = app_nh.advertiseService("app_switch_task", &Scrubber::appSwitchAutoTask, this);
		app_exit_auto_task_    = app_nh.advertiseService("app_exit_auto_task", &Scrubber::appExitAutoTask, this);
		app_enter_map_new_     = app_nh.advertiseService("app_map_new_enter", &Scrubber::appEnterMapNew, this);
		app_exit_map_new_      = app_nh.advertiseService("app_map_new_exit", &Scrubber::appExitMapNew, this);
		app_save_map_new_      = app_nh.advertiseService("app_map_new_save", &Scrubber::appSaveMapNew, this);
		app_start_path_record_ = app_nh.advertiseService("app_path_record_start", &Scrubber::appStartPathRecord, this);
		app_save_path_         = app_nh.advertiseService("app_path_save", &Scrubber::appSavePath, this);
		app_exit_path_record_  = app_nh.advertiseService("app_path_record_exit", &Scrubber::appExitPathRecord, this);
		app_enter_map_edit_    = app_nh.advertiseService("app_map_edit_enter", &Scrubber::appEnterMapEdit, this);
		app_exit_map_edit_     = app_nh.advertiseService("app_map_edit_exit", &Scrubber::appExitMapEdit, this);
		app_save_map_edit_     = app_nh.advertiseService("app_map_edit_save", &Scrubber::appSaveMapEdit, this);
		app_clean_cfg_save_    = app_nh.advertiseService("app_clean_cfg_save", &Scrubber::appSetCleanConfig, this);
		app_clean_cfg_get_     = app_nh.advertiseService("app_clean_cfg_get", &Scrubber::appGetCleanConfig, this);
		app_combine_plans_     = app_nh.advertiseService("app_sub_region_compose", &Scrubber::appCombinePlans, this);
		app_clean_record_      = app_nh.advertiseService("app_clean_record", &Scrubber::appCleanRecord, this);
		app_record_map_        = app_nh.advertiseService("app_clean_record_map", &Scrubber::appRecordMap, this);
		app_record_summary_    = app_nh.advertiseService("app_clean_record_summary", &Scrubber::appRecordSummary, this);
		app_extend_map_        = app_nh.advertiseService("app_map_extend_action", &Scrubber::appMapExtend, this);
		app_reset_map_         = app_nh.advertiseService("app_map_reset", &Scrubber::appMapReset, this);
		app_locate_            = app_nh.advertiseService("app_locate_in_map", &Scrubber::appLocate, this);
		app_modify_map_        = app_nh.advertiseService("app_point_fix", &Scrubber::appModifyMap, this);
		app_region_rename_     = app_nh.advertiseService("app_region_rename", &Scrubber::appRenameRegion, this);
		app_region_delete_     = app_nh.advertiseService("app_region_delete", &Scrubber::appDeleteRegion, this);
		//Thread
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
		app_status_.stat                 = (int8_t) cur_state_;
		app_status_.power                = battery.soc; //TODO: Get real battary status
		app_status_.total_clean_area     = float(clean_statistics.total_clean_area) / 10000;
		app_status_.total_clean_duration = float(clean_statistics.total_clean_time) / 60;
		app_status_.clean_water_level    = 70; //TODO:Get real level
		app_status_.dirty_water_level    = 50; //TODO:Get real level

		app_status_pub_.publish(app_status_);
	}

	void Scrubber::exeThread() {
		int       rate    = 2;
		int       counter = 0;
		ros::Rate r(rate);
		while (ros::ok()) {
			if (counter++ % 4 == 0) {
				pubAppMapInfo();
			} else {
				pubAppPoseInfo();
			}

			updateCurrentConfig();
			pubAppStatus();
			counter %= 2;
			ros::spinOnce();
			r.sleep();
		}
	}

	void Scrubber::updateCurrentConfig() {
		db_msgs::GetConfigScale get_config;

		get_config.request.type = db_msgs::TypeConfig::CLEAN_CONFIG;
		if (db_get_config_scale_.call(get_config)) {
			cleanCfgScale.low    = get_config.response.low;
			cleanCfgScale.medium = get_config.response.medium;
			cleanCfgScale.high   = get_config.response.high;
		}

		get_config.request.type = db_msgs::TypeConfig::SPEED_CONFIG;
		if (db_get_config_scale_.call(get_config)) {
			speedCfgScale.low    = get_config.response.low;
			speedCfgScale.medium = get_config.response.medium;
			speedCfgScale.high   = get_config.response.high;
		}


		if (clean_mech_->isEmergency()) {
			ROS_ERROR_ONCE("Emergency stop detected");
			roborock_app::StopAutoTask stop;
			appStopAutoTask(stop.request, stop.response);
			app_status_.error_code = roborock_app::RobotErrCode::ERR_EMERGENCY_STOP;
		} else {
			if (app_status_.error_code == roborock_app::RobotErrCode::ERR_EMERGENCY_STOP) {
				app_status_.error_code = 0;
			}
		}
	}

	bool Scrubber::setPathConfig(scrubber_msgs::SetPathConfigRequest& req, scrubber_msgs::SetPathConfigResponse&) {
		ROS_INFO("Scrubber set config %d", req.config_id);
		if (req.config_id == 99) { //Pause clean
			return clean_mech_->pause();
		}

		db_msgs::GetScrubberConfig config;
		Config                     clean_config;
		config.request.config_id = req.config_id;
		if (get_scrubber_config_.call(config)) {
			ROS_INFO("Got config details");
			clean_config.vacuum   = config.response.vacuum;
			clean_config.squeegee = config.response.squeegee;
			clean_config.flow     = config.response.flow;
			clean_config.brush    = config.response.brush;
			if (clean_mech_->setCleanConfig(clean_config)) {
				ROS_INFO("Scrubber set config succeeded");
				cur_config_id_ = req.config_id;
				return true;
			}
			return false;
		}

		return false;
	}


	bool Scrubber::crossBump(scrubber_msgs::CrossBumpRequest& req, scrubber_msgs::CrossBumpResponse& resp) {
		if (zones_.empty()) {
			return false;
		}

		double                            yaw;
		std::vector<geometry_msgs::Point> waypoints;
		geometry_msgs::Point              check_pose;
		geometry_msgs::PoseStamped        result_pose;
		result_pose.header.frame_id = "map";
		result_pose.header.stamp    = ros::Time::now();

		for (auto& pose : req.candidate.poses) {
			check_pose.x = pose.pose.position.x;
			check_pose.y = pose.pose.position.y;
			for (auto& zone:zones_) {
				if (inZone(zone.points, check_pose)) { //Path will cross speed bump, find way points
					waypoints = crossRoute(zone.points);
					if (waypoints.empty()) {
						ROS_WARN("Will cross bump but could not found though points");
						return false;
					}

					if (distance(check_pose, waypoints[0]) > distance(check_pose, waypoints[1])) {
						waypoints.push_back(waypoints[0]);
						waypoints.erase(waypoints.begin());
					}

					yaw = std::atan2(waypoints[1].y - waypoints[0].y, waypoints[1].x - waypoints[0].x);
					tf2::Quaternion qt;
					qt.setRPY(0, 0, yaw);
					result_pose.pose.orientation.x = qt.getX();
					result_pose.pose.orientation.y = qt.getY();
					result_pose.pose.orientation.z = qt.getZ();
					result_pose.pose.orientation.w = qt.getW();
					result_pose.pose.position.x    = waypoints[0].x;
					result_pose.pose.position.y    = waypoints[0].y;

					resp.waypoints.push_back(result_pose);

					result_pose.pose.position.x = waypoints[1].x;
					result_pose.pose.position.y = waypoints[1].y;

					resp.waypoints.push_back(result_pose);
					return true;
				}
			}
		}

		return false;
	}

	void Scrubber::currentPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
		cur_pose_ = *msg;
	}

	void Scrubber::currentState(const std_msgs::UInt8::ConstPtr& msg) {
		//Update New state
		cur_state_ = msg->data;
	}

	void Scrubber::currentMap(const std_msgs::UInt32::ConstPtr& msg) {
		map_id_ = msg->data;
	}

	void Scrubber::ultrasonic(const std_msgs::String::ConstPtr& msg) {
		int   id    = 0;
		int   width = 0;
		int   peak  = 0;
		float dist  = 0;
		std::sscanf(msg->data.c_str(), "%d,%f,%d,%d", &id, &dist, &width, &peak);

		if (id > 0 && id < 8) {
			sensor_msgs::Range range;
			range.header.frame_id = "ultrasonic_" + std::to_string(id);
			range.header.stamp    = ros::Time::now();
			range.field_of_view   = 1.0471;//60 degree
			range.min_range       = 0.1;
			range.max_range       = 3;
			range.range           = dist;
			range.radiation_type  = sensor_msgs::Range::ULTRASOUND;
			ultrasonic_pub_.publish(range);
		}
	}

	void Scrubber::updateZoneConfig() {
		if (zones_.empty()) return;
		for (Zone& zone : zones_) {
			if (inZone(zone.points, cur_pose_.pose.pose.position)) {
				if (zone_id_ == -1) {
					ROS_INFO_ONCE("In zone %d", zone.zone_id);
				}

				if (zone_id_ == zone.zone_id) return;
				saved_config_id_ = cur_config_id_;
				zone_id_         = zone.zone_id;
				scrubber_msgs::SetPathConfig config;
				config.request.config_id = zone.config_id;
				if (!setPathConfig(config.request, config.response)) {
					ROS_ERROR_ONCE("Set zone config failed");
					zone_id_ = -1; //Try next time
				}
				return;
			}
		}

		if (zone_id_ != -1) { //Moved out from a zone
			ROS_INFO("move out of zone %d", zone_id_);
			zone_id_ = -1;//Not in any zone
			scrubber_msgs::SetPathConfig config;
			config.request.config_id = saved_config_id_;
			if (!setPathConfig(config.request, config.response)) {
				ROS_ERROR("Failed to resume config");
			}
		}
	}

	bool Scrubber::appGetMapList(roborock_app::GetCleanRegionsRequest&, roborock_app::GetCleanRegionsResponse& resp) {
		//Get id and name for all maps
		db_msgs::GetList get_maps;
		get_maps.request.type = db_msgs::TypeQuery::MAP;
		if (!db_get_list_.call(get_maps)) {
			ROS_ERROR("Get map list failed");
			return false;
		}

		//For all map, get plan list belong to this map
		db_msgs::GetListInMapRequest plan_in_map;
		plan_in_map.type        = db_msgs::TypeQuery::PLAN;
		plan_in_map.by_map_name = false;
		for (auto& info:get_maps.response.contents) {
			roborock_app::CleanRegion     region;
			db_msgs::GetListInMapResponse plans;
			plan_in_map.map_id = info.id;
			db_get_list_in_map_.call(plan_in_map, plans);
			region.id               = info.id;
			region.name             = info.name;
			region.sub_region_count = plans.contents.size();
			region.create_time.data = ros::Time::now();
			region.total_area       = 999;//TODO: Query map brief info
			resp.regions.push_back(region);
		}

		return true;
	}

	bool Scrubber::appGetPlanList(roborock_app::GetCleanSubRegionsRequest& req,
	                              roborock_app::GetCleanSubRegionsResponse& resp) {
		//Get plan_id and name in given map
		db_msgs::GetListInMap get_plans;
		db_msgs::GetPlan      plan_info;
		get_plans.request.by_map_name = false;
		get_plans.request.map_id      = req.regionId;
		get_plans.request.type        = db_msgs::TypeQuery::PLAN;

		if (!db_get_list_in_map_.call(get_plans)) {
			ROS_ERROR("Get list in map failed");
			return false;
		}

		for (auto& info:get_plans.response.contents) {
			//Get plan details
			plan_info.request.plan_id = info.id;
			if (!db_get_plan_.call(plan_info)) {
				ROS_ERROR("Failed to get plan %d detail", info.id);
				continue;
			}

			//Fill out app info;
			roborock_app::CleanSubRegion sub_region;
			db_msgs::GetPathsInfo        paths_info;
			for (auto                    task:plan_info.response.tasks) {
				sub_region.path_ids.push_back(task.path_id);
				paths_info.request.paths.push_back(task.path_id);
			}

			if (!db_paths_info_.call(paths_info)) {
				ROS_ERROR("Failed to get path info for plan %d", info.id);
			}

			sub_region.id                      = info.id;
			sub_region.name                    = info.name;
			sub_region.type                    = plan_info.response.type;
			sub_region.create_time.data        = ros::Time(plan_info.response.create_time);
			//TODO: Area & time calculation
			sub_region.total_area              = (paths_info.response.upper_bound.x
			                                      - paths_info.response.lower_bound.x)
			                                     * (paths_info.response.upper_bound.y
			                                        - paths_info.response.lower_bound.y);
			sub_region.estimate_clean_duration = sub_region.total_area / 8;
			resp.subRegions.push_back(sub_region);
		}

		return true;
	}

	bool Scrubber::appGetMap(roborock_app::GetRegionMapRequest& req, roborock_app::GetRegionMapResponse& resp) {
		resp.mapInfo.regionId = req.regionId;
		return getMapInfo(req.regionId, resp.mapInfo, true);
	}

	bool Scrubber::appGetPaths(roborock_app::GetRegionPathsRequest& req,
	                           roborock_app::GetRegionPathsResponse& resp) {

		db_msgs::GetListInMap get_plans;
		get_plans.request.map_id = req.regionId;
		get_plans.request.type   = db_msgs::TypeQuery::PLAN;

		if (!db_get_list_in_map_.call(get_plans)) {
			ROS_ERROR("Unable to get path list for map: %d", req.regionId);
			return false;
		}

		db_msgs::GetPlan        get_plan;
		db_msgs::GetTask        get_task;
		roborock_app::CleanPath app_path;

		for (auto& plan: get_plans.response.contents) {
			get_plan.request.plan_id = plan.id;
			if (!db_get_plan_.call(get_plan)) {
				ROS_ERROR("Failed to get plan %d detail", plan.id);
				continue;
			}

			for (auto& task: get_plan.response.tasks) {
				get_task.request.task_id = task.task_id;
				if (!ros::service::call("scrubber_database/getTask", get_task)) {
					ROS_ERROR("Failed to get task %d detail", task.task_id);
					continue;
				}

				navPathtoAppPath(get_task.response.path, app_path);
				app_path.id = task.path_id;
				resp.paths.push_back(app_path);
			}
		}

		return true;
	}

	bool Scrubber::appStartAutoTask(roborock_app::StartAutoTaskRequest& req,
	                                roborock_app::StartAutoTaskResponse& resp) {
		if (cur_state_ == States::AUTOMATIC_CLEAN || cur_state_ == States::AUTOMATIC_GOTO) {
			ROS_ERROR("Cancel previous auto mission first before sending a new one");
			resp.errCode = 3;
			return false;
		}


		if (map_id_ != req.regionId) {
			db_msgs::GetMap get_map;
			get_map.request.map_id       = req.regionId;
			get_map.request.pub_this_map = true;
			db_get_map_.call(get_map);
			if (get_map.response.map.data.empty()) {
				ROS_ERROR("Got empty map %d", get_map.response.map_id);
				resp.errCode = 1; //No map error
				return false;
			}
		}


		if (cur_state_ == States::PAUSE) {
			std_srvs::Empty ept;
			if (last_plan_id_ == req.subRegionId
			    && ros::service::call("launcher/resume", ept)) { //Resume from pause
				app_status_.stat_for_app = roborock_app::StatForAppCode::CLEAN;
				return true;
			}

			ros::service::call("launcher/goto_cancel", ept); //cancel possible goto mission
			launcher_->cancelGoal(); //cancel last mission
		}

		updateZone();

		if (cur_state_ == States::AUTOMATIC) {
			launcher_msgs::LauncherGoal goal;
			goal.mission.type      = States::AUTOMATIC;
			goal.mission.id        = req.subRegionId;
			goal.mission.config    = appCfgIndexToCfgMode(req.configIndex);
			goal.mission.exe_times = req.cleanTimes;
			sendGoal(goal);
			last_plan_id_ = goal.mission.id;
			resp.errCode             = 0;
			app_status_.stat_for_app = roborock_app::StatForAppCode::CLEAN;
			return true;
		}

		if (cur_state_ != States::PREPARING) {
			launcher_msgs::PrepareNode pn;
			pn.request.target_state = States::AUTOMATIC;
			if (!ros::service::call("launcher/prepare", pn)) {
				ROS_ERROR("Prepare failed try relocation");
				resp.errCode = 2;
				return false;
			}

			launcher_msgs::LauncherGoal goal;
			goal.mission.type      = States::AUTOMATIC;
			goal.mission.id        = req.subRegionId;
			goal.mission.config    = appCfgIndexToCfgMode(req.configIndex);
			goal.mission.exe_times = req.cleanTimes;
			sendGoal(goal);
			last_plan_id_ = goal.mission.id;
			resp.errCode = 0;
			return true;
		}

		//Still in preparing
		ROS_ERROR("Failed to start auto task current state: %d , try relocation again?", cur_state_);
		resp.errCode = 4;
		return false;
	}

	bool Scrubber::appStartPathRecord(roborock_app::StartPathRecordRequest& req,
	                                  roborock_app::StartPathRecordResponse& resp) {
		if (map_id_ != req.regionId) {
			db_msgs::GetMap get_map;
			get_map.request.map_id       = req.regionId;
			get_map.request.pub_this_map = true;
			db_get_map_.call(get_map);
			if (get_map.response.map.data.empty()) {
				ROS_ERROR("Got empty map %d", get_map.response.map_id);
				resp.errCode = 1; //No map error
				return false;
			}
		}

		if (cur_state_ == States::PAUSE) { //Resume from pause
			std_srvs::Empty ept;
			//Resume from pause
			return ros::service::call("launcher/resume", ept);
		}

		if (cur_state_ == States::TRACKING) {
			launcher_->cancelGoal();
			ros::Duration(0.2).sleep(); //Time for end last tracking
			launcher_msgs::LauncherGoal goal;
			goal.mission.type = States::TRACKING;
			sendGoal(goal);
			resp.errCode = 0;
			return true;
		}

		if (cur_state_ != States::PREPARING) { //Other states to tracking, shouldn't happen
			launcher_msgs::PrepareNode pn;
			pn.request.target_state = States::TRACKING;
			if (!ros::service::call("launcher/prepare", pn)) {
				ROS_ERROR("Prepare failed in start tracking");
				resp.errCode = 2;
				return false;
			}

			launcher_msgs::LauncherGoal goal;
			goal.mission.type = States::TRACKING;
			sendGoal(goal);
			return true;
		}

		//Still in preparing

		ROS_ERROR("Failed to start tracking current state: %d , try relocation again?", cur_state_);
		resp.errCode = 4;
		return false;
	}

	bool Scrubber::appEnterMapNew(roborock_app::EnterMapNewRequest& req, roborock_app::EnterMapNewResponse& resp) {
		app_pub_map_ = true;

		//Prepare for drawing new map
		launcher_msgs::PrepareNode pn;
		pn.request.target_state = States::MAPPING;
		if (!ros::service::call("launcher/prepare", pn)) {
			ROS_ERROR("Prepare failed try relocation");
			return false;
		}

		//Start mapping
		launcher_msgs::LauncherGoal goal;
		goal.mission.type = States::MAPPING;
		sendGoal(goal);
		//Enable manual control
		if (!clean_mech_->setManualConfig()) {
			ROS_WARN("Failed to enable manual control");
			//TODO: return false
		}

		resp.errCode = 0;
		return true;
	}

	bool Scrubber::appExitMapNew(roborock_app::ExitMapNewRequest&, roborock_app::ExitMapNewResponse& resp) {
		return toIdle();
	}

	bool Scrubber::appSaveMapNew(roborock_app::SaveMapNewRequest& req, roborock_app::SaveMapNewResponse& resp) {
		db_msgs::AddMap add_map;
		add_map.request.name = req.mapName;
		if (!ros::service::call("scrubber_database/addMap", add_map)) {
			resp.errCode = 1;
			return false;
		}

		resp.errCode  = 0;
		resp.regionId = add_map.response.map_id;
		return true;
	}

	bool Scrubber::appEnterMapEdit(roborock_app::EnterMapEditRequest& req, roborock_app::EnterMapEditResponse& resp) {


		return true;
	}

	bool Scrubber::appExitMapEdit(roborock_app::ExitMapEditRequest& req, roborock_app::ExitMapEditResponse& resp) {
		if (cur_state_ != States::IDLE) {
			toIdle();
		}

		resp.errCode = 0;
		return true;
	}

	bool Scrubber::appSaveMapEdit(roborock_app::SaveMapEditRequest& req, roborock_app::SaveMapEditResponse& resp) {
		//Delete keep out zone
		std::vector<uint8_t>  del_zone_types = {db_msgs::TypeZone::KEEPOUT,
		                                        db_msgs::TypeZone::SLOWDOWN,
		                                        db_msgs::TypeZone::PARKING,
		                                        db_msgs::TypeZone::CHARGING,
		                                        db_msgs::TypeZone::DUMP,
		                                        db_msgs::TypeZone::REFILL,
		                                        db_msgs::TypeZone::REPD,         //Refill Plus Dump
		                                        db_msgs::TypeZone::ELEVATOR};
		db_msgs::GetListInMap del_zones;
		db_msgs::GetZone      get_zone;
		del_zones.request.map_id = req.info.regionId;
		del_zones.request.type   = db_msgs::TypeQuery::ZONE;

		for (auto& type : del_zone_types) {
			del_zones.request.filter = type;
			if (!db_get_list_in_map_.call(del_zones)) {
				ROS_ERROR("Failed to get del zone candidate list, map id %d type %d", req.info.regionId, type);
				continue;
			}

			if (type == db_msgs::TypeZone::KEEPOUT) {
				for (auto del_it = del_zones.response.contents.begin(); del_it != del_zones.response.contents.end();) {
					bool drop = false;
					get_zone.request.zone_id = del_it->id;
					if (!db_get_zone_.call(get_zone)) {
						ROS_WARN("Save map edit fail to get existing keepout zone info %d", del_it->id);
						del_it++;
						continue;
					}

					for (auto ins_it = req.info.forbiddens.begin(); ins_it != req.info.forbiddens.end();) {
						if (rectEqual(pathToRect(get_zone.response.path), *ins_it)) {
							ROS_INFO("Found same keep out zone!");
							drop   = true; //No need to delete old one
							ins_it = req.info.forbiddens.erase(ins_it);//Nothing updates for this one
							continue;
						}

						ins_it++;
					}

					if (drop) {
						del_it = del_zones.response.contents.erase(del_it); //Remove it from list
						continue;
					}

					del_it++;
				}
			}


			db_msgs::Delete del;
			for (auto& info: del_zones.response.contents) {
				del.request.type = db_msgs::TypeQuery::ZONE;
				del.request.id   = info.id;
				if (!db_delete_.call(del)) {
					ROS_ERROR("Fail to delete zone, id %d", del.request.id);
				}
			}
		}

		//Clear Old data?
		//std_srvs::Empty ept;
		//ros::service::call("scrubber_database/clearOldZones", ept);

		//Add new ones
		db_msgs::AddZone add_zone;
		add_zone.request.map_id = req.info.regionId;
		geometry_msgs::PoseStamped pose;
		db_msgs::AddPath           add_path;
		add_path.request.by_topic             = false;
		add_path.request.map_id               = req.info.regionId;
		pose.header.frame_id                  = "map";
		add_path.request.path.header.frame_id = "map";
		pose.header.stamp                     = ros::Time::now();
		add_path.request.path.header.stamp    = ros::Time::now();
		pose.pose.orientation.w               = 1;

		//Add virtual wall
		for (auto& line : req.info.virtualWalls) {
			add_path.request.path.poses.clear();
			pose.pose.position.x = line.start.x;
			pose.pose.position.y = line.start.y;
			add_path.request.path.poses.push_back(pose);
			pose.pose.position.x = line.end.x;
			pose.pose.position.y = line.end.y;
			add_path.request.path.poses.push_back(pose);
			add_path.request.type = db_msgs::TypePath::BOUNDARY;
			add_path.request.name = "VirtualWall_@" + std::to_string(ros::Time::now().toSec());
			if (!db_add_path_.call(add_path)) {
				ROS_ERROR("Unable to add virtual wall as a path");
				continue;
			}

			add_zone.request.path_id = add_path.response.path_id;
			add_zone.request.name    = add_path.request.name;
			add_zone.request.type    = db_msgs::TypeZone::KEEPOUT;
			if (!db_add_zone_.call(add_zone)) {
				ROS_ERROR("Unable to add virtual wall as a zone");
				continue;
			}
		}

		//Add Keep Out Zone
		for (auto& rect: req.info.forbiddens) {
			add_path.request.path.poses.clear();
			pose.pose.position.x = rect.first.x;
			pose.pose.position.y = rect.first.y;
			add_path.request.path.poses.push_back(pose);
			pose.pose.position.x = rect.second.x;
			pose.pose.position.y = rect.second.y;
			add_path.request.path.poses.push_back(pose);
			pose.pose.position.x = rect.third.x;
			pose.pose.position.y = rect.third.y;
			add_path.request.path.poses.push_back(pose);
			pose.pose.position.x = rect.fourth.x;
			pose.pose.position.y = rect.fourth.y;
			add_path.request.path.poses.push_back(pose);
			add_path.request.type = db_msgs::TypePath::BOUNDARY;
			add_path.request.name = "KEEPOUT_" + std::to_string(ros::Time::now().toSec());
			if (!db_add_path_.call(add_path)) {
				ROS_ERROR("Unable to add keep out zone as a path");
				continue;
			}

			add_zone.request.path_id = add_path.response.path_id;
			add_zone.request.name    = add_path.request.name;
			add_zone.request.type    = db_msgs::TypeZone::KEEPOUT;
			if (!db_add_zone_.call(add_zone)) {
				ROS_ERROR("Unable to add keep out zone as a zone");
				continue;
			}
		}

		//Add Speed Bumps
		for (auto& rect: req.info.speedBumps) {
			add_path.request.path.poses.clear();
			pose.pose.position.x = rect.first.x;
			pose.pose.position.y = rect.first.y;
			add_path.request.path.poses.push_back(pose);
			pose.pose.position.x = rect.second.x;
			pose.pose.position.y = rect.second.y;
			add_path.request.path.poses.push_back(pose);
			pose.pose.position.x = rect.third.x;
			pose.pose.position.y = rect.third.y;
			add_path.request.path.poses.push_back(pose);
			pose.pose.position.x = rect.fourth.x;
			pose.pose.position.y = rect.fourth.y;
			add_path.request.path.poses.push_back(pose);
			add_path.request.type = db_msgs::TypePath::BOUNDARY;
			add_path.request.name = "SPEEDBUMP@" + std::to_string(ros::Time::now().toSec());
			if (!db_add_path_.call(add_path)) {
				ROS_ERROR("Unable to add speed bump zone as a path");
				continue;
			}

			add_zone.request.path_id = add_path.response.path_id;
			add_zone.request.name    = add_path.request.name;
			add_zone.request.type    = db_msgs::TypeZone::SLOWDOWN;
			if (!db_add_zone_.call(add_zone)) {
				ROS_ERROR("Unable to add speed bump zone as a zone");
			}
		}

		//Reset init zone for plan and map
		db_msgs::UpdateInit update_init;
		for (auto& locateRect : req.info.locateRects) {
			bool success;
			update_init.request.id       = locateRect.subRegionId == -1 ? req.info.regionId : locateRect.subRegionId;
			update_init.request.new_init = rectToPath(locateRect.rect);
			if (locateRect.subRegionId == -1) { //Global init zone for map
				success = ros::service::call("scrubber_database/updateMapInit", update_init);
			} else {
				success = db_update_plan_init_.call(update_init);
			}

			if (!success) {
				ROS_ERROR("Fail to update map %d plan %d", req.info.regionId, locateRect.subRegionId);
			}
		}

		//Update mark points
		db_msgs::SetMarkPoint set_point;
		set_point.request.map_id                  = req.info.regionId;
		set_point.request.pose.pose.orientation.w = 1;
		for (auto& point:req.info.markPoints) {
			set_point.request.type = appMkpTypeToTypeZone(point.type);
			ROS_INFO("Mark point type %d", set_point.request.type);
			if (set_point.request.type == 0) {
				continue;
			}

			set_point.request.pose.pose.position.x = point.pos.x;
			set_point.request.pose.pose.position.y = point.pos.y;

			if (!ros::service::call("scrubber_database/setMarkPoint", set_point)) {
				ROS_WARN("Fail to update mark point type %d", set_point.request.type);
			}
		}

		//Conflict resolve
		conflictResolve(req.info);

		//New autofill zones for test
		std::string    name;
		nav_msgs::Path boundary;
		uint32_t       plan_id;
		for (auto& rect:req.info.pathFillRects) {
			name     = "AUTOFILL" + std::to_string(ros::Time::now().toSec());
			boundary = rectToPath(rect);
			autoFillPlan(name, req.info.regionId, boundary, plan_id);
		}

		return true;
	}

	bool Scrubber::getMapInfo(int map_id, roborock_app::RosMapInfo& map_info, bool pub_map) {
		return getMapInfo(map_id, map_info, true, pub_map);
	}

	bool Scrubber::getMapInfo(int map_id, roborock_app::RosMapInfo& map_info, bool fill_path, bool pub_map) {
		//Get Map
		db_msgs::GetMap get_map;
		get_map.request.pub_this_map = pub_map;
		get_map.request.map_id       = map_id;
		if (!db_get_map_.call(get_map)) {
			ROS_ERROR("Get map failed id:%d", get_map.request.map_id);
			return false;
		}

		//map_info.map = get_map.response.map;
		compressMap(get_map.response.map, map_info.map);
		//Get Virtual Wall & Keep Out Zone
		db_msgs::GetListInMap keep_out;
		keep_out.request.type   = db_msgs::TypeQuery::ZONE;
		keep_out.request.map_id = map_id;
		keep_out.request.filter = db_msgs::TypeZone::KEEPOUT;
		if (!db_get_list_in_map_.call(keep_out)) {
			ROS_ERROR("Failed to get keep out zone list");
			return false;
		}

		for (auto& info:keep_out.response.contents) {
			db_msgs::GetZone get_zone;
			get_zone.request.zone_id = info.id;
			if (!db_get_zone_.call(get_zone)) {
				ROS_ERROR("Failed to get zone info");
				continue;
			}

			if (get_zone.response.path.poses.size() < 2) {
				ROS_ERROR("Got zone with less than 2 points, this shouldn't happen");
				continue;
			}

			if (get_zone.response.path.poses.size() == 2) { //Virtual wall
				roborock_app::Line line;
				line.start.x = get_zone.response.path.poses[0].pose.position.x;
				line.start.y = get_zone.response.path.poses[0].pose.position.y;
				line.end.x   = get_zone.response.path.poses[1].pose.position.x;
				line.end.y   = get_zone.response.path.poses[1].pose.position.y;
				map_info.virtualWalls.push_back(line);
			}

			if (get_zone.response.path.poses.size() == 4) { //Keep Out Rectangle
				roborock_app::Rect rect;
				rect.first.x  = get_zone.response.path.poses[0].pose.position.x;
				rect.first.y  = get_zone.response.path.poses[0].pose.position.y;
				rect.second.x = get_zone.response.path.poses[1].pose.position.x;
				rect.second.y = get_zone.response.path.poses[1].pose.position.y;
				rect.third.x  = get_zone.response.path.poses[2].pose.position.x;
				rect.third.y  = get_zone.response.path.poses[2].pose.position.y;
				rect.fourth.x = get_zone.response.path.poses[3].pose.position.x;
				rect.fourth.y = get_zone.response.path.poses[3].pose.position.y;
				map_info.forbiddens.push_back(rect);
			}
		}

		//Get speed bump zone
		db_msgs::GetListInMap speed_bump;
		speed_bump.request.type   = db_msgs::TypeQuery::ZONE;
		speed_bump.request.map_id = map_id;
		speed_bump.request.filter = db_msgs::TypeZone::SLOWDOWN;
		if (!db_get_list_in_map_.call(speed_bump)) {
			ROS_ERROR("Failed to get slow down zone list");
			return false;
		}

		for (auto& info:speed_bump.response.contents) {
			db_msgs::GetZone get_zone;
			get_zone.request.zone_id = info.id;
			if (!db_get_zone_.call(get_zone)) {
				ROS_ERROR("Failed to get zone info");
				continue;
			}

			if (get_zone.response.path.poses.size() < 2) {
				ROS_ERROR("Got zone with less than 2 points, this shouldn't happen");
				continue;
			}

			if (get_zone.response.path.poses.size() == 4) { //Speed bump Rectangle
				roborock_app::Rect rect;
				rect.first.x  = get_zone.response.path.poses[0].pose.position.x;
				rect.first.y  = get_zone.response.path.poses[0].pose.position.y;
				rect.second.x = get_zone.response.path.poses[1].pose.position.x;
				rect.second.y = get_zone.response.path.poses[1].pose.position.y;
				rect.third.x  = get_zone.response.path.poses[2].pose.position.x;
				rect.third.y  = get_zone.response.path.poses[2].pose.position.y;
				rect.fourth.x = get_zone.response.path.poses[3].pose.position.x;
				rect.fourth.y = get_zone.response.path.poses[3].pose.position.y;
				map_info.speedBumps.push_back(rect);
			}
		}

		//Get relocation zones
		if (fill_path) {
			db_msgs::GetListInMap    relo_list;
			db_msgs::GetZone         get_zone;
			db_msgs::GetPlan         get_plan;
			roborock_app::LocateRect locate_rect;
			relo_list.request.map_id = map_id;
			relo_list.request.type   = db_msgs::TypeQuery::ZONE;
			relo_list.request.filter = db_msgs::TypeZone::INIT_GLOBAL;//General relocation zone
			if (!db_get_list_in_map_.call(relo_list)) {
				ROS_ERROR("Failed to get global relocation zone_id");
			}

			for (auto& info: relo_list.response.contents) {
				locate_rect.subRegionId  = -1;
				get_zone.request.zone_id = info.id;
				if (!db_get_zone_.call(get_zone)) {
					ROS_ERROR("Failed to get global relocation zone %d", info.id);
					continue;
				}

				locate_rect.rect = pathToRect(get_zone.response.path);
				map_info.locateRects.push_back(locate_rect);
			}

			relo_list.request.type   = db_msgs::TypeQuery::PLAN;
			relo_list.request.filter = 0;//No filter
			relo_list.response.contents.clear();
			if (!db_get_list_in_map_.call(relo_list)) {
				ROS_ERROR("Failed to get plan id for map %d while looking for relocation zone binding to plan", map_id);
			}

			for (auto& plan: relo_list.response.contents) {
				get_plan.request.plan_id = plan.id;
				if (!db_get_plan_.call(get_plan)) {
					ROS_ERROR("Failed to get plan content");
					continue;
				}

				get_zone.request.zone_id = get_plan.response.init_zone_id;
				if (!db_get_zone_.call(get_zone)) {
					ROS_ERROR("Failed to get relocation zone %d for plan %d ", get_zone.request.zone_id, plan.id);
					continue;
				}

				locate_rect.subRegionId = plan.id;
				locate_rect.rect        = pathToRect(get_zone.response.path);
				map_info.locateRects.push_back(locate_rect);
			}

//			//Get all route path
//
//			roborock_app::GetRegionPaths get_paths;
//			get_paths.request.regionId = map_id;
//			if (!appGetPaths(get_paths.request, get_paths.response)) {
//				ROS_ERROR("Failed to get all paths in map %d", map_id);
//			}
//
//			map_info.paths = get_paths.response.paths;
		}
		//Get mark points
		db_msgs::GetMarkPoint   get_mkp;
		roborock_app::MarkPoint app_mkp;
		get_mkp.request.map_id = map_id;
		//App mark point types
		std::vector<uint8_t> mkp_type = {
				roborock_app::MarkPointType::PARK,
				roborock_app::MarkPointType::CHARGE,
				roborock_app::MarkPointType::DRAIN,
				roborock_app::MarkPointType::WATER
		};

		for (auto& app_type: mkp_type) {
			get_mkp.request.type = appMkpTypeToTypeZone(app_type);
			if (!ros::service::call("scrubber_database/getMarkPoint", get_mkp)) {
				continue;
			}

			app_mkp.type  = app_type;
			app_mkp.pos.x = get_mkp.response.pose.pose.position.x;
			app_mkp.pos.y = get_mkp.response.pose.pose.position.y;
			map_info.markPoints.push_back(app_mkp);
		}

		if (!map_info.markPoints.empty()) {
			ROS_INFO("mark points size: %ld 1st type %d x %.2f y %.2f z%.2f",
			         map_info.markPoints.size(), map_info.markPoints[0].type,
			         map_info.markPoints[0].pos.x, map_info.markPoints[0].pos.y, map_info.markPoints[0].pos.z);
		}

		return true;
	}

	bool Scrubber::appLocate(roborock_app::LocateRequest& req, roborock_app::LocateResponse& resp) {
		ROS_WARN("APP Locate map_id: %d plan_id: %d", req.regionId, req.subRegionId);
		//If we had target map
		if (map_id_ != req.regionId) {
			db_msgs::GetMap get_map;
			get_map.request.map_id       = req.regionId;
			get_map.request.by_map_name  = false;
			get_map.request.pub_this_map = true;
			db_get_map_.call(get_map);
			if (get_map.response.map.data.empty()) {
				ROS_ERROR("Got empty map %d", get_map.response.map_id);
				resp.errCode = 1; //No map error
				return false;
			}
		}

		//Prepare node for relocation
		launcher_msgs::PrepareNode pn;
		pn.request.target_state = req.locateType;
		if (!ros::service::call("launcher/prepare", pn)) {
			ROS_ERROR("Prepare %d failed, current state %d", req.locateType, cur_state_);
			resp.errCode = 2;
			return false;
		}

		db_msgs::GetPlan get_plan;
		db_msgs::GetZone get_zone;
		amcl::RectPara   search_area;

		if (req.subRegionId != -1) { //Plan's init zone
			get_plan.request.plan_id = req.subRegionId;
			if (!db_get_plan_.call(get_plan)) {
				ROS_ERROR("Failed to get plan %d info when relocating", req.subRegionId);
				resp.errCode = 3;
				return false;
			}

			get_zone.request.zone_id = get_plan.response.init_zone_id;
		} else { //Map's init zone
			db_msgs::GetListInMap map_init;
			map_init.request.map_id = req.regionId;
			map_init.request.type   = db_msgs::TypeQuery::ZONE;
			map_init.request.filter = db_msgs::TypeZone::INIT_GLOBAL;
			if (!db_get_list_in_map_.call(map_init)) {
				ROS_ERROR("Failed to map %d when relocating", req.regionId);
				resp.errCode = 3;
				return false;
			}

			if (map_init.response.contents.empty()) {
				ROS_ERROR("Got empty init zone for map %d when relocating", req.regionId);
			}

			get_zone.request.zone_id = map_init.response.contents[0].id;
		}

		if (!db_get_zone_.call(get_zone)) {
			ROS_ERROR("Failed to get relocate zone info when relocating");
			resp.errCode = 3;
			return false;
		}

		if (get_zone.response.path.poses.empty()) {
			ROS_ERROR("Got empty zone");
			resp.errCode = 4;
			return false;
		}

		search_area.request.rect_max_x = get_zone.response.path.poses[0].pose.position.x;
		search_area.request.rect_min_x = get_zone.response.path.poses[0].pose.position.x;
		search_area.request.rect_max_y = get_zone.response.path.poses[0].pose.position.y;
		search_area.request.rect_min_y = get_zone.response.path.poses[0].pose.position.y;

		//Find min & max...
		for (auto& pose : get_zone.response.path.poses) {
			if (pose.pose.position.x > search_area.request.rect_max_x) {
				search_area.request.rect_max_x = pose.pose.position.x;
			}

			if (pose.pose.position.x < search_area.request.rect_min_x) {
				search_area.request.rect_min_x = pose.pose.position.x;
			}

			if (pose.pose.position.y > search_area.request.rect_max_y) {
				search_area.request.rect_max_y = pose.pose.position.y;
			}

			if (pose.pose.position.y < search_area.request.rect_min_y) {
				search_area.request.rect_min_y = pose.pose.position.y;
			}
		}

		ros::Duration(0.2).sleep();//Might need this time to get everything ready
		return ros::service::call("launcher/relocation", search_area);
	}

	void Scrubber::currentMapData(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
		//if (!app_pub_map_) return;
		original_map_ = *msg;
		map_info_.size_x   = msg->info.width;
		map_info_.size_y   = msg->info.height;
		map_info_.origin_x = msg->info.origin.position.x;
		map_info_.origin_y = msg->info.origin.position.y;
		map_info_.scale    = msg->info.resolution;
	}

	void Scrubber::pubAppPoseInfo() {
		if (cur_state_ == States::IDLE || cur_state_ == States::MANUAL || cur_state_ == States::PREPARING)
			return;

		app_map_info_.laserData.clear();
		app_map_info_.map = {};
		geometry_msgs::TransformStamped transformStamped;
		double                          roll, pitch, yaw;
		try {
			transformStamped = tf_buffer_.lookupTransform("map", "base_link",
			                                              ros::Time(0));
			app_map_info_.robotPosition.x = transformStamped.transform.translation.x;
			app_map_info_.robotPosition.y = transformStamped.transform.translation.y;
			tf2::Quaternion qt(transformStamped.transform.rotation.x,
			                   transformStamped.transform.rotation.y,
			                   transformStamped.transform.rotation.z,
			                   transformStamped.transform.rotation.w);
			tf2::Matrix3x3(qt).getRPY(roll, pitch, yaw);
			app_map_info_.robotPosition.z = yaw;
		} catch (tf2::TransformException& ex) {
			ROS_WARN_ONCE("%s", ex.what());
		}

		geometry_msgs::PointStamped laser_point, map_point;
		laser_point.header = original_laser_.header;
		double      angle        = original_laser_.angle_min - original_laser_.angle_increment;
		std::string target_frame = "map";
		for (auto& range: original_laser_.ranges) {
			angle += original_laser_.angle_increment;
			if (range < original_laser_.range_min || range > original_laser_.range_max) continue;

			laser_point.point.x = cos(angle) * range;
			laser_point.point.y = sin(angle) * range;

			try {
				tf_buffer_.transform(laser_point, map_point, target_frame);
				app_map_info_.laserData.push_back(floor(map_point.point.x / map_info_.scale));
				app_map_info_.laserData.push_back(floor(map_point.point.y / map_info_.scale));
				//ROS_INFO("Laser on map %.2f, %.2f", map_point.point.x, map_point.point.y);
			} catch (tf2::TransformException& ex) {
				ROS_WARN_ONCE("No transform from laser to map %s", ex.what());
				break;
			}
		}

		app_map_info_pub_.publish(app_map_info_);
	}

	void Scrubber::pubAppMapInfo() {
		if (cur_state_ == States::IDLE || cur_state_ == States::MANUAL || cur_state_ == States::PREPARING)
			return;

		if (app_pub_map_) {
			compressMap(original_map_, app_map_info_.map);
			app_map_info_pub_.publish(app_map_info_);
		}
	}

	void Scrubber::trackingCallback(const nav_msgs::Path::ConstPtr& msg) {
		origianl_path_ = *msg;
		roborock_app::CleanPath app_path;
		navPathtoAppPath(origianl_path_, app_path);
		app_map_info_.paths.clear();
		app_map_info_.paths.push_back(app_path);
	}

	void Scrubber::laserScan(const sensor_msgs::LaserScan::ConstPtr& msg) {
		if (cur_state_ == States::IDLE || cur_state_ == States::MANUAL || cur_state_ == States::PREPARING)
			return;

		original_laser_ = *msg;
	}

	bool Scrubber::appDeleteRegion(roborock_app::DeleteRegionRequest& req, roborock_app::DeleteRegionResponse& resp) {
		db_msgs::Delete del;
		std_srvs::Empty ept;
		if (req.subRegionId == -1) { //delete map
			del.request.type = db_msgs::TypeQuery::MAP;
			del.request.id   = req.regionId;
			return db_delete_.call(del);
		} else { //delete plan
			del.request.type = db_msgs::TypeQuery::PLAN;
			del.request.id   = req.subRegionId;
			return db_delete_.call(del);
			//&& ros::service::call("scrubber_database/clearOldPlans", ept);
		}
	}

	bool Scrubber::appRenameRegion(roborock_app::RenameRegionRequest& req, roborock_app::RenameRegionResponse& resp) {
		db_msgs::Rename rename;
		rename.request.new_name = req.newName;
		if (req.subRegionId == -1) {
			rename.request.type = db_msgs::TypeQuery::MAP;
			rename.request.id   = req.regionId;
		} else {
			rename.request.type = db_msgs::TypeQuery::PLAN;
			rename.request.id   = req.subRegionId;
		}

		return ros::service::call("scrubber_database/rename", rename);
	}

	bool Scrubber::updateStatistics(scrubber_msgs::UpdateStatisticsRequest& req,
	                                scrubber_msgs::UpdateStatisticsResponse&) {
		clean_statistics.total_clean_area += req.clean_area_increment;
		clean_statistics.total_clean_time += req.clean_time_increment;
		app_status_.clean_duration += req.clean_time_increment;
		app_status_.clean_area += req.clean_area_increment;
		return true;
	}

	bool Scrubber::appSwitchAutoTask(roborock_app::SwitchTaskRequest&, roborock_app::SwitchTaskResponse&) {
		if (goal_active_) {
			launcher_->cancelGoal();
		}

		return true;
	}

	bool Scrubber::appStopAutoTask(roborock_app::StopAutoTaskRequest&, roborock_app::StopAutoTaskResponse& resp) {
		if (cur_state_ == States::AUTOMATIC_GOTO || cur_state_ == States::AUTOMATIC_CLEAN) {
			std_srvs::Empty ept;
			app_status_.stat_for_app = roborock_app::StatForAppCode::CLEAN_PAUSE;
			return ros::service::call("launcher/pause", ept);
		}

		if (goal_active_) {
			launcher_->cancelGoal();
		}

		return true;
	}

	bool Scrubber::appExitAutoTask(roborock_app::ExitAutoTaskRequest&, roborock_app::ExitAutoTaskResponse&) {
		last_plan_id_ = 0;

		return toIdle();
	}

	bool Scrubber::appSavePath(roborock_app::SavePathRequest& req, roborock_app::SavePathResponse& resp) {
		if (!validPath(original_map_, origianl_path_)) {
			ROS_WARN("Path in obstacle or unknown space");
			resp.errCode = 2;
			return false;
		}

		db_msgs::AddPath add_path;
		db_msgs::Task    task;
		db_msgs::AddPlan add_plan;
		uint32_t         plan_id;
		add_path.request.path   = origianl_path_;
		add_path.request.type   = req.type == roborock_app::PathType::DEMO ?
		                          db_msgs::TypePath::ROUTE : db_msgs::TypePath::BOUNDARY;
		add_path.request.map_id = req.regionId;
		add_path.request.name   = req.name;
		if (!db_add_path_.call(add_path)) {
			ROS_ERROR("App failed to add path");
			resp.errCode = roborock_app::CmdErrCode::DUPLICATE_MAP_NAME;
			return false;
		}

		if (req.type == roborock_app::PathType::AUTO_FILL) {
			if (!autoFillPlan(req.name, req.regionId, origianl_path_, plan_id)) {
				ROS_ERROR("Failed to generate auto fill plan");
				return false;
			}
		} else if (req.type == roborock_app::PathType::DEMO) { //Path for plan
			task.type               = 1;
			task.path_id            = add_path.response.path_id;
			task.config_id          = 1;
			add_plan.request.map_id = req.regionId;
			add_plan.request.name   = req.name;
			add_plan.request.ordered_tasks.push_back(task);
			if (!db_add_plan_.call(add_plan)) {
				ROS_ERROR("Failed to generate clean plan for path %s", req.name.c_str());
				resp.errCode = 1;
				return false;
			}
		}
		//TODO:Path for Keep Out zone, Speed Bumper
		return true;
	}

	bool Scrubber::toIdle() {
		app_pub_map_  = false;
		app_map_info_ = {};
		app_status_.clean_area     = 0;
		app_status_.clean_duration = 0;
		launcher_->cancelGoal();
		while (goal_active_) {
			ROS_WARN_ONCE("Wait for ending last mission...");
			ros::Duration(0.1).sleep();
			ros::spinOnce();
		}

		launcher_msgs::PrepareNode pn;
		pn.request.target_state = States::IDLE;
		if (!ros::service::call("launcher/prepare", pn)) {
			ROS_ERROR("Failed to IDLE");
			return false;
		}

		clean_mech_->setManualConfig();
		return true;
	}

	bool Scrubber::autoFillPlan(std::string& name, uint32_t map_id, nav_msgs::Path& cur_path, uint32_t& plan_id) {
		db_msgs::AddPlan add_plan;
		db_msgs::AddPath add_path;
		db_msgs::Task    task;
		if (cur_path.poses.empty()) {
			ROS_ERROR("No zone data for auto generate");
			return false;
		}

		//Start movebase for costmap info
		launcher_msgs::PrepareNode pn;
		pn.request.target_state = States::AUTOMATIC;
		if (!ros::service::call("launcher/prepare", pn)) {
			ROS_ERROR("Prepare %d failed, current state %d", States::AUTOMATIC, cur_state_);
			return false;
		}

		ros::Duration(0.5).sleep();//Wait for costmap ready TODO:Find a better way
		//Generate auto fill paths
		path_coverage::GetPathInZone gen_path;
		gen_path.request.zone = cur_path;
		if (!ros::service::call("getPathInZone", gen_path)) {
			ROS_ERROR("Failed to generate auto fill paths");
			return false;
		}

		//Save paths and generate task list;
		add_plan.request.name   = name;
		add_plan.request.map_id = map_id;
		add_plan.request.ordered_tasks.clear();
		add_path.request.map_id = map_id;
		int i = 1;
		for (auto& path: gen_path.response.paths) {
			add_path.request.by_topic = false;
			add_path.request.type     = db_msgs::TypePath::ROUTE;
			add_path.request.name     = name + "_Auto_FILL_" + std::to_string(i++)
			                            + "@" + std::to_string(ros::Time::now().toSec());
			add_path.request.path     = path;
			if (!db_add_path_.call(add_path)) {
				ROS_ERROR("App failed to add auto generated path");
				return false;
			}

			task.type      = 1;
			task.config_id = 1;
			task.path_id   = add_path.response.path_id;
			add_plan.request.ordered_tasks.push_back(task);
		}

		if (!db_add_plan_.call(add_plan)) {
			ROS_ERROR("Failed to generate clean plan for clean zone %s", name.c_str());
			return false;
		}

		//Save Zone
		add_path.request.type = db_msgs::TypePath::BOUNDARY;
		add_path.request.name = name + "_Auto_FILL_BOUNDARY@" + std::to_string(ros::Time::now().toSec());
		add_path.request.path = cur_path;
		if (!db_add_path_.call(add_path)) {
			ROS_ERROR("Failed to save zone boundary");
			return false;
		}

		db_msgs::AddZone clean_zone;
		clean_zone.request.type = db_msgs::TypeZone::CLEAN;
		clean_zone.request.name = name + "_Auto_FILL_ZONE@" + std::to_string(ros::Time::now().toSec());;
		clean_zone.request.map_id    = map_id;
		clean_zone.request.path_id   = add_path.response.path_id;
		clean_zone.request.config_id = 1;
		if (!db_add_zone_.call(clean_zone)) {
			ROS_ERROR("Failed to save path as a clean zone");
			return false;
		}

		//Bind plan and zone
		db_msgs::BindAutofill bind;
		bind.request.plan_id = add_plan.response.plan_id;
		bind.request.zone_id = clean_zone.response.zone_id;
		if (!ros::service::call("scrubber_database/bindAutofill", bind)) {
			ROS_ERROR("Failed to bind clean plan %d and clean zone %d", bind.request.plan_id, bind.request.zone_id);
			return false;
		}

		plan_id = add_plan.response.plan_id;
		return toIdle();
	}


	bool Scrubber::appManualConfig(roborock_app::ManualConfigRequest& req, roborock_app::ManualConfigResponse& resp) {
		if (cur_state_ != States::MANUAL && req.configType != roborock_app::ConfigType::MANUAL_START) {
			ROS_WARN("Should start manual before set other configs");
			return false;
		}
		Config config;
		config.squeegee = config.flow = config.vacuum = config.brush = -1;

		if (req.configType == roborock_app::ConfigType::MANUAL_START) {
			if (cur_state_ != States::IDLE) {
				toIdle();
			}

			clean_mech_->setManualConfig(); //TODO: Determin success or not
			app_status_.walk_speed     = clean_mech_->getManualConfig().speed;
			app_status_.walk_direction = clean_mech_->getManualConfig().direction ?
			                             roborock_app::Direction::BACKWARD : roborock_app::Direction::FORWARD;
			launcher_msgs::LauncherGoal goal;
			goal.mission.type = States::MANUAL;
			sendGoal(goal);
			resp.errCode = 0;
		} else if (req.configType == roborock_app::ConfigType::MANUAL_STOP) {

			config.squeegee = config.flow = config.vacuum = config.brush = 0;
			if (!clean_mech_->setCleanConfig(config)) {
				ROS_ERROR("Failed to reset manual configs");
				return false;
			}

			//manual_config.request.enable = false;
			clean_mech_->setManualConfig(); //TODO: Determin success or not
			app_status_.water_suck_stat = app_status_.brush_disc_stat = roborock_app::UpDown::UP;
			app_status_.water_suck_speed
					= app_status_.brush_disc_speed
					= app_status_.water_pump_speed
					= roborock_app::Speed::STOP;
			app_status_.walk_speed     = clean_mech_->getManualConfig().speed;
			app_status_.walk_direction = roborock_app::Direction::FORWARD;
			launcher_->cancelGoal();
		} else if (req.configType == roborock_app::ConfigType::BRUSH_DISC_STAT) {
			if (!clean_mech_->setBrush(req.intValue == roborock_app::UpDown::DOWN)) {
				return false;
			}

			//Setting success, update status to app
			app_status_.brush_disc_stat  = req.intValue;
			app_status_.brush_disc_speed = roborock_app::Speed::STOP;
		} else if (req.configType == roborock_app::ConfigType::WATER_SUCK_STAT) {
			if (!(clean_mech_->setSqueegee(req.intValue == roborock_app::UpDown::DOWN) &&
			    clean_mech_->setVacuum(req.intValue == roborock_app::UpDown::DOWN? -1: 0))) {
				return false;
			}

			app_status_.water_suck_stat  = req.intValue;
			app_status_.water_suck_speed = roborock_app::Speed::STOP;
		} else if (req.configType == roborock_app::ConfigType::BRUSH_DISC_SPEED ||
		           req.configType == roborock_app::ConfigType::WATER_PUMP_SPEED ||
		           req.configType == roborock_app::ConfigType::WATER_SUCK_SPEED) {
			int cfg_scale = 0;
			//ROS_INFO("Speed level %d", req.intValue);
			switch (req.intValue) {
				case roborock_app::Speed::STOP:
					cfg_scale = 0;
					break;
				case roborock_app::Speed::SLOW:
					cfg_scale = cleanCfgScale.low;
					break;
				case roborock_app::Speed::MIDDLE:
					cfg_scale = cleanCfgScale.medium;
					break;
				case roborock_app::Speed::FAST:
					cfg_scale = cleanCfgScale.high;
					break;
				default:
					ROS_WARN("Unknown brush config type");
			}

			switch (req.configType) {
				case roborock_app::ConfigType::BRUSH_DISC_SPEED:
					clean_mech_->setBrush(cfg_scale);
					break;
				case roborock_app::ConfigType::WATER_PUMP_SPEED:
					clean_mech_->setFlow(cfg_scale);
					break;
				case roborock_app::ConfigType::WATER_SUCK_SPEED:
					clean_mech_->setSqueegee(cfg_scale == 0 ? 0 : 1); //Put down squeegee if vacuum is on
					clean_mech_->setVacuum(cfg_scale);
					break;
			}

			switch (req.configType) {//Update app status on success
				case roborock_app::ConfigType::BRUSH_DISC_SPEED:
					app_status_.brush_disc_speed = req.intValue;
					app_status_.brush_disc_stat  = req.intValue == roborock_app::Speed::STOP ?
					                               roborock_app::UpDown::UP : roborock_app::UpDown::DOWN;
					break;
				case roborock_app::ConfigType::WATER_PUMP_SPEED:
					app_status_.water_pump_speed = req.intValue;
					break;
				case roborock_app::ConfigType::WATER_SUCK_SPEED:
					app_status_.water_suck_speed = req.intValue;
					app_status_.water_suck_stat  = req.intValue == roborock_app::Speed::STOP ?
					                               roborock_app::UpDown::UP : roborock_app::UpDown::DOWN;
					break;
			}

		} else if (req.configType == roborock_app::ConfigType::WALK_DIRECTION) { //TODO:SPEED & DIRECTION
			clean_mech_->setManualConfig(true,
			                             req.intValue == roborock_app::Direction::BACKWARD ? 1 : 0,
			                             app_status_.walk_speed); //TODO: Determin success or not
			app_status_.walk_direction = req.intValue;
		} else if (req.configType == roborock_app::ConfigType::WALK_SPEED) {
			clean_mech_->setManualConfig(true,
			                             app_status_.walk_direction == roborock_app::Direction::BACKWARD ? 1 : 0,
			                             req.floatValue); //TODO: Determin success or not
			app_status_.walk_speed = req.floatValue;
		} else if (req.configType == roborock_app::ConfigType::SOUND_PLAY) {
			//TODO: No hardware support yet
		} else {
			ROS_ERROR("Unknown app config type");
			return false;
		}

		return true;
	}

	bool Scrubber::appGetCleanConfig(roborock_app::GetCleanConfigRequest& req,
	                                 roborock_app::GetCleanConfigResponse& resp) {
		int8_t                 clean_mode = appCfgIndexToCfgMode(req.index);
		db_msgs::GetConfigMode get_mode;
		if (clean_mode == -1) {
			ROS_WARN("Try to get unknown clean mode");
			return false;
		}

		get_mode.request.config_mode = clean_mode;
		if (!ros::service::call("scrubber_database/getConfigMode", get_mode)) {
			ROS_ERROR("Failed to get clean mode index %d", req.index);
			return false;
		}

		resp.config.index            = req.index;
		resp.config.walk_speed       = roborock_app::Speed::MIDDLE;
		resp.config.brush_disc_stat  = get_mode.response.brush > 1 ? roborock_app::UpDown::DOWN
		                                                           : roborock_app::UpDown::UP;
		resp.config.brush_disc_speed = dbSpeedToAppSpeed(get_mode.response.brush);
		resp.config.water_suck_stat  = get_mode.response.vacuum > 1 ? roborock_app::UpDown::DOWN
		                                                            : roborock_app::UpDown::UP;
		resp.config.water_suck_speed = dbSpeedToAppSpeed(get_mode.response.vacuum);
		resp.config.water_pump_speed = dbSpeedToAppSpeed(get_mode.response.flow);
		//resp.config.fan_speed = dbSpeedToAppSpeed(get_mode.response.vacuum);
		return true;
	}

	bool Scrubber::appSetCleanConfig(roborock_app::CleanConfigRequest& req,
	                                 roborock_app::CleanConfigResponse& resp) {
		int8_t clean_mode = appCfgIndexToCfgMode(req.index);
		if (clean_mode == -1 || clean_mode == db_msgs::ConfigMode::VACUUM
		    || clean_mode == db_msgs::ConfigMode::ECU_WASH) {
			ROS_WARN("Try to modify unknown mode or read only mode, mode index %d", req.index);
			return false;
		}

		db_msgs::SetConfigMode set_mode;
		set_mode.request.config_mode = clean_mode;
		set_mode.request.squeegee    = -1;
		switch (req.type) {
			case roborock_app::ConfigType::BRUSH_DISC_STAT:
				set_mode.request.brush = req.intValue == roborock_app::UpDown::UP ? db_msgs::ConfigValueID::OFF : -1;
				break;
			case roborock_app::ConfigType::BRUSH_DISC_SPEED:
				set_mode.request.brush = appSpeedTodbSpeed(req.intValue);
				break;
			case roborock_app::ConfigType::WATER_SUCK_STAT: //squeegee work with vacuum
				set_mode.request.vacuum   = req.intValue == roborock_app::UpDown::UP ? db_msgs::ConfigValueID::OFF : -1;
				set_mode.request.squeegee = req.intValue == roborock_app::UpDown::UP ? 0 : -1;
				break;
			case roborock_app::ConfigType::WATER_SUCK_SPEED:
				set_mode.request.vacuum   = appSpeedTodbSpeed(req.intValue);
				set_mode.request.squeegee = set_mode.request.vacuum > 1 ? 1 : 0;
				break;
			case roborock_app::ConfigType::WATER_PUMP_SPEED:
				set_mode.request.flow = appSpeedTodbSpeed(req.intValue);
				break;
			case roborock_app::ConfigType::WALK_SPEED:
				//TODO: speed config
				break;
			default:
				ROS_WARN("Got unknown config type when clean configs %d", req.type);
		}

		return ros::service::call("scrubber_database/setConfigMode", set_mode);
	}

	bool Scrubber::appMapExtend(roborock_app::MapExtendActionRequest& req,
	                            roborock_app::MapExtendActionResponse& resp) {
		return true;
	}

	bool Scrubber::appMapReset(roborock_app::ResetMapRequest& req, roborock_app::ResetMapResponse& resp) {
		db_msgs::ResetMap         reset;
		roborock_app::SaveMapEdit clear;
		reset.request.map_id = req.regionId;
		if (!ros::service::call("scrubber_database/resetMap", reset)) {
			ROS_WARN("Failed to reset map");
			return false;
		}

		clear.request.info.regionId = req.regionId;
		return appSaveMapEdit(clear.request, clear.response);
	}

	bool Scrubber::appCombinePlans(roborock_app::SubRegionComposeRequest& req,
	                               roborock_app::SubRegionComposeResponse& resp) {
//		db_msgs::GetPlan get_plan;
		db_msgs::AddPlan add_plan;
		db_msgs::Task    new_task;
//		std::set<uint32_t> path_set;
		//Get all the paths of original plan
//		for(auto plan_id: req.subRegionIds){
//			get_plan.request.plan_id = plan_id;
//			if(!ros::service::call("scrubber_database/getPlan", get_plan)){
//				ROS_WARN("Failed to get plan %d", plan_id);
//				continue;
//			}
//
//			for(auto task: get_plan.response.tasks){
//				path_set.insert(task.path_id);
//			}
//		}
//
//		//Generate combined plan
		add_plan.request.type   = roborock_app::SubRegionType::COMPOSITE;
		add_plan.request.map_id = req.regionId;
		add_plan.request.name   = req.subRegionName;

		for (auto path: req.subRegionIds) { //TODO: Currently it's all PATH_IDs
			new_task.path_id = path;
			ROS_INFO("Path id %d", path);
			add_plan.request.ordered_tasks.push_back(new_task);
		}

		return db_add_plan_.call(add_plan);
	}

	bool Scrubber::appExitPathRecord(roborock_app::ExitPathRecordRequest&, roborock_app::ExitPathRecordResponse& resp) {
		return toIdle();
	}

	bool Scrubber::appCleanRecord(roborock_app::GetCleanRecordRequest& req,
	                              roborock_app::GetCleanRecordResponse& resp) {
		db_msgs::GetCleanRecords  records;
		roborock_app::CleanRecord record;
		records.request.first = req.firstRecord;
		records.request.count = req.recordCount;
		if (!ros::service::call("scrubber_database/getCleanRecords", records)) {
			ROS_WARN("Failed to get clean records");
			return false;
		}

		for (auto& data:records.response.records) {
			record.id              = data.record_id;
			record.sub_region_name = data.plan_name;
			record.clean_config    = cfgModeToappCfgIndex(data.config_id);
			record.clean_area      = float(data.area) / 10000; //cm^2->m^2
			record.clean_time.data = ros::Time(data.create_time - data.duration);
			record.clean_duration  = float(data.duration) / 60;
			record.water_used      = data.water_comsumption;
			record.power_cost      = data.power_comsumption;
			record.cover_rate      = data.cover_rate;
			resp.cleanRecords.push_back(record);
		}

		return true;
	}

	bool Scrubber::appRecordMap(roborock_app::GetCleanRecordMapRequest& req,
	                            roborock_app::GetCleanRecordMapResponse& resp) {
		db_msgs::GetRecordInfo  record_info;
		db_msgs::GetPath        get_path;
		roborock_app::CleanPath app_path;
		record_info.request.record_id = req.id;
		if (!ros::service::call("scrubber_database/getRecordInfo", record_info)) {
			ROS_WARN("Failed to get info for record %d", req.id);
			return false;
		}

		get_path.request.path_id = record_info.response.path_id;
		if (!db_get_path_.call(get_path)) {
			ROS_WARN("Failed to get app_path %d", get_path.request.path_id);
			return false;
		}

		navPathtoAppPath(get_path.response.path, app_path);
		app_path.id = record_info.response.path_id;
		if (!getMapInfo(record_info.response.map_id, resp.mapInfo, false, false)) {
			ROS_ERROR("Failed to get record map %d", record_info.response.map_id);
		}

		resp.mapInfo.paths.push_back(app_path);
		return true;
	}

	bool Scrubber::appRecordSummary(roborock_app::GetCleanRecordSummaryRequest&,
	                                roborock_app::GetCleanRecordSummaryResponse& resp) {
		db_msgs::GetRecordSummary summary;
		if (!ros::service::call("scrubber_database/getRecordSummary", summary)) {
			ROS_WARN("Failed to get clean record summary");
			return false;
		}

		resp.summary.total_clean_area     = float(summary.response.total_clean_area) / 10000; //cm^2 -> m^2
		resp.summary.total_clean_times    = summary.response.total_clean_times;
		resp.summary.total_clean_duration = float(summary.response.total_clean_durition) / 60; //s -> min
		return true;
	}

	void Scrubber::launcherDone(const actionlib::SimpleClientGoalState& state,
	                            const launcher_msgs::LauncherResultConstPtr& result) {
		ROS_INFO("Mission %d done state [%s]", result->result.id, state.toString().c_str());
		app_status_.stat_for_app = roborock_app::StatForAppCode::CLEAN_STOPPED;
		if (state != state.SUCCEEDED) {
			ROS_WARN("Mission failed notify app");
			app_status_.error_code = roborock_app::RobotErrCode::ERR_TASK_FAIL;
		}

		goal_active_ = false;
		app_map_info_.paths.clear();
	}

	void Scrubber::sendGoal(launcher_msgs::LauncherGoal& goal) {
		if (goal_active_) {
			ROS_WARN("Another goal is active!");
			launcher_->cancelGoal();
			while (goal_active_) {
				ros::Duration(0.1).sleep();
				ros::spinOnce();
			}
		}

		launcher_->sendGoal(goal,
		                    boost::bind(&Scrubber::launcherDone, this, _1, _2),
		                    actionlib::SimpleActionClient<launcher_msgs::LauncherAction>::SimpleActiveCallback(),
		                    actionlib::SimpleActionClient<launcher_msgs::LauncherAction>::SimpleFeedbackCallback()
		);

		goal_active_ = true;
	}

	bool Scrubber::navPathtoAppPath(nav_msgs::Path& nav_path, roborock_app::CleanPath& app_path) {
		app_path.path.clear();
		int path_length = nav_path.poses.size();

		if (path_length == 0) {
			return false;
		}

		int i    = 0;
		int skip = 5; //Skip some pose

		while (i < path_length - 1) {
			app_path.path.push_back(floor(nav_path.poses[i].pose.position.x / map_info_.scale));
			app_path.path.push_back(floor(nav_path.poses[i].pose.position.y / map_info_.scale));
			i += skip;
		}

		//Add last pose
		app_path.path.push_back(floor(nav_path.poses[path_length - 1].pose.position.x / map_info_.scale));
		app_path.path.push_back(floor(nav_path.poses[path_length - 1].pose.position.y / map_info_.scale));

		app_path.pathLength = (int32_t) app_path.path.size() / 2; //May need compress in future

		return true;
	}

	bool Scrubber::appGetPlanPaths(roborock_app::GetPathsRequest& req, roborock_app::GetPathsResponse& resp) {
		ROS_WARN("App get paths map id: %d, plan id: %d", req.regionId, req.subRegionId);
		db_msgs::GetPlan        get_plan;
		db_msgs::GetPath        get_path;
		roborock_app::CleanPath app_path;
		get_plan.request.plan_id = req.subRegionId;
		if (!db_get_plan_.call(get_plan)) {
			ROS_ERROR("Failed to get plan info id:%d", req.subRegionId);
			return false;
		}

		for (auto& task:get_plan.response.tasks) {
			get_path.request.path_id = task.path_id;
			if (!db_get_path_.call(get_path)) {
				ROS_WARN("Failed to get path %d", task.path_id);
				continue;
			}

			app_path.id = task.path_id;
			navPathtoAppPath(get_path.response.path, app_path);
			resp.paths.push_back(app_path);
		}

		return true;
	}

	bool Scrubber::conflictResolve(roborock_app::RosMapInfo& info) {
		if (info.forbiddens.empty() && info.virtualWalls.empty()) {//No possible conflict
			return true;
		}

		for (auto& wall: info.virtualWalls) {
			roborock_app::Rect wall_zone;
			wall_zone.first  = wall.start;
			wall_zone.second = wall.end;
			wall_zone.third  = wall.end;
			wall_zone.fourth = wall.start;
			wall_zone.third.x += 0.04;
			wall_zone.third.y += 0.05;
			wall_zone.fourth.x += 0.05;
			wall_zone.fourth.y += 0.04;
			info.forbiddens.push_back(wall_zone);
		}

		db_msgs::GetListInMap plan_list;
		plan_list.request.map_id = info.regionId;
		plan_list.request.type   = db_msgs::TypeQuery::PLAN;
		if (!db_get_list_in_map_.call(plan_list)) {
			ROS_INFO("No plan in this map yet");
			return true;
		}

		//Get autofill zones and check whether there's a conflict
		db_msgs::GetAutofillZone af_zone;
		db_msgs::GetZone         get_zone;
		db_msgs::Delete          del_plan;
		db_msgs::GetPlan         get_plan;
		db_msgs::GetPlan         get_new_plan;
		db_msgs::UpdateInit      update_plan_init;

		std::vector<geometry_msgs::Point> keepout;
		for (auto& plan_info: plan_list.response.contents) {
			af_zone.request.plan_id = plan_info.id;
			if (!db_get_autofill_zone_.call(af_zone))//Not a autofill plan
				continue;

			//Check keep out zones
			for (auto& forbidden : info.forbiddens) {
				keepout.clear();
				keepout.push_back(forbidden.first);
				keepout.push_back(forbidden.second);
				keepout.push_back(forbidden.third);
				keepout.push_back(forbidden.fourth);
				if (overlap(af_zone.response.zone, keepout)) {
					ROS_WARN("Keep out zone overlap with clean zones, regenerate cleaning path...");
					//Get old plan info
					get_plan.request.plan_id = plan_info.id;
					if (!db_get_plan_.call(get_plan)) {
						ROS_ERROR("Update confilict plan failed to get old plan info");
						break;
					}

					get_zone.request.zone_id = get_plan.response.init_zone_id;
					get_zone.response.path.poses.clear();
					if (!db_get_zone_.call(get_zone)) {
						ROS_ERROR("Update confilict plan failed to get old init zone");
						break;
					}

					//Delete conflict plan
					del_plan.request.id   = plan_info.id;
					del_plan.request.type = db_msgs::TypeQuery::PLAN;
					db_delete_.call(del_plan);
					//Generate new plan
					uint32_t       new_plan_id;
					nav_msgs::Path bound  = pointsToPath(af_zone.response.zone);
					if (!autoFillPlan(plan_info.name, info.regionId, bound, new_plan_id)) {
						ROS_ERROR("Failed to update conflict plan");
						break;
					}

					//Get new plan info
					get_new_plan.request.plan_id = new_plan_id;
					if (!db_get_plan_.call(get_new_plan)) {
						ROS_ERROR("Failed to get new plan info, combined plan will not be updated");
						break;
					}

					//Update plan init
					update_plan_init.request.id       = new_plan_id;
					update_plan_init.request.new_init = get_zone.response.path;
					if (!db_update_plan_init_.call(update_plan_init)) {
						ROS_WARN("Update conflict plan failed to update init zone");
					}

					if (!updateCombinedPlan(info.regionId, get_plan.response.tasks, get_new_plan.response.tasks)) {
						ROS_ERROR("Failed to update combined plans");
					}

					break;
				}
			}
		}

		return true;
	}

	bool Scrubber::updateCombinedPlan(uint32_t map_id, std::vector<db_msgs::Task>& old_tasks,
	                                  std::vector<db_msgs::Task>& new_tasks) {
		if (old_tasks.empty() || new_tasks.empty()) {
			ROS_ERROR("Got empty tasks for updating combined plan");
			return false;
		}

		std::vector<uint32_t> bad_paths;
		db_msgs::GetPlan      get_plan;
		bool                  update;

		for (auto& task:old_tasks) {
			bad_paths.push_back(task.path_id);
		}

		db_msgs::GetListInMap combined_plans;
		combined_plans.request.map_id = map_id;
		combined_plans.request.type   = db_msgs::TypeQuery::PLAN;
		combined_plans.request.filter = db_msgs::TypePlan::COMPOSITE;
		if (!db_get_list_in_map_.call(combined_plans)) {
			ROS_INFO("May be no combined plan");
			return false;
		}

		for (auto& info: combined_plans.response.contents) {
			update = false;
			get_plan.request.plan_id = info.id;
			if (!db_get_plan_.call(get_plan)) {
				ROS_WARN("Update combined plan failed to get plan %d", info.id);
				continue;
			}

			std::vector<db_msgs::Task> tasks = get_plan.response.tasks;
			for (auto                  it    = tasks.begin(); it != tasks.end();) {
				if (std::find(bad_paths.begin(), bad_paths.end(), it->path_id) != bad_paths.end()) {
					update = true;
					it     = tasks.erase(it);
					continue;
				}

				it++;
			}

			if (update) {
				for (auto& replace_task: new_tasks) {
					tasks.push_back(replace_task);
				}

				db_msgs::UpdatePlan update_plan;
				update_plan.request.plan_id       = info.id;
				update_plan.request.map_id        = map_id;
				update_plan.request.ordered_tasks = tasks;

				if (!db_update_plan_.call(update_plan)) {
					ROS_ERROR("Failed to update combined plan %d", info.id);
				}
			}

		}

		return true;
	}


	void Scrubber::batteryStatus(const std_msgs::UInt8MultiArray::ConstPtr& msg) {
		if (msg->data.size() < 15) {
			return;
		}

		ROS_INFO_ONCE("Got battery info");
		battery.soc            = msg->data[13];
		battery.charging_state = msg->data[12];
	}

	void Scrubber::fluidLevel(const std_msgs::UInt8MultiArray::ConstPtr& msg) {
		if (msg->data.empty()) return;
		ROS_INFO_ONCE("Got fluid info");
		fluid_level_.clean_water = msg->data[0];
	}

	bool Scrubber::appModifyMap(roborock_app::PointFixRequest& req, roborock_app::PointFixResponse& resp) {
		MapInfo            info = {};
		db_msgs::GetMap    get_map;
		db_msgs::UpdateMap upd_map;
		get_map.request.map_id = req.regionId;
		if (!db_get_map_.call(get_map)) {
			ROS_WARN("Modify map failed to get map data");
			return false;
		}

		info.scale    = get_map.response.map.info.resolution;
		info.origin_x = get_map.response.map.info.origin.position.x;
		info.origin_y = get_map.response.map.info.origin.position.y;
		info.size_x   = get_map.response.map.info.width;
		info.size_y   = get_map.response.map.info.height;

		std::vector<geometry_msgs::Point> bound = rectToPointArray(req.rect);
		geometry_msgs::Point              lower_bound, upper_bound, test_point;
		diagonal(bound, lower_bound, upper_bound);

		for (double x = lower_bound.x; x <= upper_bound.x;) {
			for (double y = lower_bound.y; y <= upper_bound.y;) {
				test_point.x = x;
				test_point.y = y;
				if (inZone(bound, test_point)) {
					int g_x = info.worldXtoGridX(test_point.x);
					int g_y = info.worldYtoGridY(test_point.y);
					int idx = g_x + g_y * info.size_x;
					if (idx > 0 && idx < get_map.response.map.data.size()) {
						get_map.response.map.data[idx] = req.fixType == roborock_app::PointFixType::SET ? 100 : 0;
					}
				}
				y += info.scale;
			}
			x += info.scale;
		}

		upd_map.request.map_id = req.regionId;
		upd_map.request.map    = get_map.response.map;
		return db_update_map_.call(upd_map);
	}

	void Scrubber::updateZone() {
		zones_.clear();
		//-1 means zone or data for the current map were modified, will re publish map id
		// to refresh zones
		if (map_id_ == -1) return;

		db_msgs::GetListInMap zones_in_map;
		zones_in_map.request.type   = db_msgs::TypeQuery::ZONE;
		zones_in_map.request.map_id = map_id_;
		zones_in_map.request.filter = db_msgs::TypeZone::SLOWDOWN;
		if (!db_get_list_in_map_.call(zones_in_map)) {
			ROS_ERROR("Unable to get zone info for map %d", map_id_);
			return;
		}

		if (zones_in_map.response.contents.empty()) {
			ROS_INFO("No zone in map %d", map_id_);
			return;
		}

		db_msgs::GetZone get_zone;
		get_zone.request.by_zone_name = false;
		for (db_msgs::Content& i : zones_in_map.response.contents) {
			Zone zone;
			get_zone.request.zone_id = i.id;
			ROS_INFO("Zone_id : %d\n Zone_name : %s", i.id, i.name.c_str());
			if (!db_get_zone_.call(get_zone)) {
				ROS_ERROR("Fail to get zone %s", i.name.c_str());
				continue;
			}

			if (get_zone.response.type == db_msgs::TypeZone::SLOWDOWN) {
				zone.zone_id   = i.id;
				zone.type      = get_zone.response.type;
				zone.config_id = get_zone.response.config_id;
				for (auto& pose: get_zone.response.path.poses) {
					geometry_msgs::Point point;
					point.x = pose.pose.position.x;
					point.y = pose.pose.position.y;
					zone.points.push_back(point);
				}

				zones_.push_back(zone);
			}
		}
	}

	bool Scrubber::validPath(nav_msgs::OccupancyGrid& map, nav_msgs::Path& path) {
		int     map_size = map.data.size();
		MapInfo info     = {};
		info.origin_x = map.info.origin.position.x;
		info.origin_y = map.info.origin.position.y;
		info.size_x   = map.info.width;
		info.size_y   = map.info.height;
		info.scale    = map.info.resolution;

		for (auto& pose: path.poses) {
			int g_x = info.worldXtoGridX(pose.pose.position.x);
			int g_y = info.worldYtoGridY(pose.pose.position.y);
			int idx = g_x + g_y * info.size_x;
			if (idx > 0 && idx < map_size && map.data[idx] != 0) {
				return false;
			}
		}

		return true;
	}
}


















