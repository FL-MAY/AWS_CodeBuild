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
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <scrubber_msgs/CrossBump.h>
#include "scrubber/scrubber_util.h"
#include "scrubber/Mechanism.h"

namespace rock::scrubber::scrubber {
	typedef launcher_msgs::State States;

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

		void currentMapData(const nav_msgs::OccupancyGrid::ConstPtr& msg);

		void laserScan(const sensor_msgs::LaserScan::ConstPtr& msg);

		void ultrasonic(const std_msgs::String::ConstPtr& msg);

		void trackingCallback(const nav_msgs::Path::ConstPtr& msg);

		void batteryStatus(const std_msgs::UInt8MultiArray::ConstPtr& msg);

		void fluidLevel(const std_msgs::UInt8MultiArray::ConstPtr& msg);

		void launcherDone(const actionlib::SimpleClientGoalState& state,
		                  const launcher_msgs::LauncherResultConstPtr& result);

		void sendGoal(launcher_msgs::LauncherGoal& goal);

		void updateZoneConfig();

		void updateZone();

		void updateCurrentConfig();

		void pubVelocityLimit();

		void pubAppStatus();

		void pubAppMapInfo();

		void pubAppPoseInfo();

		void exeThread();

		bool toIdle();

		bool crossBump(scrubber_msgs::CrossBumpRequest& req,
		               scrubber_msgs::CrossBumpResponse& resp);

		bool setPathConfig(scrubber_msgs::SetPathConfigRequest& req,
		                   scrubber_msgs::SetPathConfigResponse&);

		bool updateStatistics(scrubber_msgs::UpdateStatisticsRequest&,
		                      scrubber_msgs::UpdateStatisticsResponse&);

		bool appGetMapList(roborock_app::GetCleanRegionsRequest&,
		                   roborock_app::GetCleanRegionsResponse& resp);

		bool appGetPlanList(roborock_app::GetCleanSubRegionsRequest& req,
		                    roborock_app::GetCleanSubRegionsResponse& resp);

		bool appDeleteRegion(roborock_app::DeleteRegionRequest& req,
		                     roborock_app::DeleteRegionResponse& resp);

		bool appRenameRegion(roborock_app::RenameRegionRequest& req,
		                     roborock_app::RenameRegionResponse& resp);

		bool appGetMap(roborock_app::GetRegionMapRequest& req, roborock_app::GetRegionMapResponse& resp);

		bool appModifyMap(roborock_app::PointFixRequest& req, roborock_app::PointFixResponse& resp);

		bool appGetPaths(roborock_app::GetRegionPathsRequest& req, roborock_app::GetRegionPathsResponse& resp);

		bool appGetPlanPaths(roborock_app::GetPathsRequest& req, roborock_app::GetPathsResponse& resp);

		bool appStartAutoTask(roborock_app::StartAutoTaskRequest& req, roborock_app::StartAutoTaskResponse& resp);

		bool appStopAutoTask(roborock_app::StopAutoTaskRequest&, roborock_app::StopAutoTaskResponse&);

		bool appSwitchAutoTask(roborock_app::SwitchTaskRequest&, roborock_app::SwitchTaskResponse&);

		bool appExitAutoTask(roborock_app::ExitAutoTaskRequest&, roborock_app::ExitAutoTaskResponse&);

		bool appStartPathRecord(roborock_app::StartPathRecordRequest&, roborock_app::StartPathRecordResponse&);

		bool appSavePath(roborock_app::SavePathRequest&, roborock_app::SavePathResponse&);

		bool appExitPathRecord(roborock_app::ExitPathRecordRequest&, roborock_app::ExitPathRecordResponse&);

		bool appEnterMapNew(roborock_app::EnterMapNewRequest& req, roborock_app::EnterMapNewResponse& resp);

		bool appExitMapNew(roborock_app::ExitMapNewRequest&, roborock_app::ExitMapNewResponse& resp);

		bool appSaveMapNew(roborock_app::SaveMapNewRequest& req, roborock_app::SaveMapNewResponse& resp);

		bool appEnterMapEdit(roborock_app::EnterMapEditRequest& req, roborock_app::EnterMapEditResponse& resp);

		bool appExitMapEdit(roborock_app::ExitMapEditRequest& req, roborock_app::ExitMapEditResponse& resp);

		bool appSaveMapEdit(roborock_app::SaveMapEditRequest& req, roborock_app::SaveMapEditResponse& resp);

		bool appLocate(roborock_app::LocateRequest& req, roborock_app::LocateResponse& resp);

		bool appManualConfig(roborock_app::ManualConfigRequest& req, roborock_app::ManualConfigResponse& resp);

		bool appSetCleanConfig(roborock_app::CleanConfigRequest& req, roborock_app::CleanConfigResponse& resp);

		bool appGetCleanConfig(roborock_app::GetCleanConfigRequest& req, roborock_app::GetCleanConfigResponse& resp);

		bool appMapExtend(roborock_app::MapExtendActionRequest& req, roborock_app::MapExtendActionResponse& resp);

		bool appMapReset(roborock_app::ResetMapRequest& req, roborock_app::ResetMapResponse& resp);

		bool appCleanRecord(roborock_app::GetCleanRecordRequest& req, roborock_app::GetCleanRecordResponse& resp);

		bool appRecordSummary(roborock_app::GetCleanRecordSummaryRequest&,
		                      roborock_app::GetCleanRecordSummaryResponse& resp);

		bool appRecordMap(roborock_app::GetCleanRecordMapRequest& req, roborock_app::GetCleanRecordMapResponse& resp);

		bool appCombinePlans(roborock_app::SubRegionComposeRequest& req, roborock_app::SubRegionComposeResponse& resp);

		bool getMapInfo(int map_id, roborock_app::RosMapInfo& map_info, bool pub_map = false);

		bool getMapInfo(int map_id, roborock_app::RosMapInfo& map_info, bool fill_path, bool pub_map);

		bool autoFillPlan(std::string& name, uint32_t map_id, nav_msgs::Path& boundary, uint32_t& plan_id);

		bool conflictResolve(roborock_app::RosMapInfo& info);

		bool updateCombinedPlan(uint32_t map_id,
		                        std::vector<db_msgs::Task>& old_tasks, std::vector<db_msgs::Task>& new_tasks);

		bool navPathtoAppPath(nav_msgs::Path&, roborock_app::CleanPath&);

		bool validPath(nav_msgs::OccupancyGrid& map, nav_msgs::Path& path);

		uint8_t                    cur_state_;
		int32_t                    map_id_;
		int32_t                    zone_id_;
		int32_t                    saved_config_id_;
		int32_t                    cur_config_id_;
		int32_t                    last_plan_id_;
		double                     max_x_config_;
		double                     max_x_base_;
		bool                       clean_on_;
		bool                       app_pub_map_;
		bool                       goal_active_;
		MapInfo                    map_info_;
		FluidLevel                 fluid_level_;
		Battery                    battery;
		Config                     cur_config_;
		CleanStatistics            clean_statistics;
		ConfigScale                cleanCfgScale;
		ConfigScale                speedCfgScale;
		std::vector<Zone>          zones_;
		tf2_ros::Buffer            tf_buffer_;
		tf2_ros::TransformListener tf2_;
		std::shared_ptr<Mechanism> clean_mech_;

		ros::Publisher     vel_limit_;
		ros::Publisher     app_status_pub_;
		ros::Publisher     app_map_info_pub_;
		ros::Publisher     ultrasonic_pub_;
		ros::Subscriber    path_sub_;
		ros::Subscriber    state_sub_;
		ros::Subscriber    emg_sub_;
		ros::Subscriber    base_vel_sub_;
		ros::Subscriber    pose_sub_;
		ros::Subscriber    map_id_sub_;
		ros::Subscriber    map_data_sub_;
		ros::Subscriber    ultrasonic_sub_;
		ros::Subscriber    scan_sub_;
		ros::Subscriber    battery_sub_;
		ros::Subscriber    fluid_sub_;
		ros::ServiceServer cross_bump_srv_;
		ros::ServiceServer path_config_srv_;
		ros::ServiceServer update_statistics_srv_;
		ros::ServiceServer app_get_map_list_;
		ros::ServiceServer app_get_plan_list_;
		ros::ServiceServer app_get_map_;
		ros::ServiceServer app_get_paths_;
		ros::ServiceServer app_get_plan_paths_;
		ros::ServiceServer app_region_rename_;
		ros::ServiceServer app_region_delete_;
		ros::ServiceServer app_start_auto_task_;
		ros::ServiceServer app_stop_auto_task_;
		ros::ServiceServer app_switch_auto_task_;
		ros::ServiceServer app_exit_auto_task_;
		ros::ServiceServer app_start_path_record_;
		ros::ServiceServer app_save_path_;
		ros::ServiceServer app_exit_path_record_;
		ros::ServiceServer app_enter_map_new_;
		ros::ServiceServer app_exit_map_new_;
		ros::ServiceServer app_save_map_new_;
		ros::ServiceServer app_extend_map_;
		ros::ServiceServer app_reset_map_;
		ros::ServiceServer app_enter_map_edit_;
		ros::ServiceServer app_exit_map_edit_;
		ros::ServiceServer app_save_map_edit_;
		ros::ServiceServer app_locate_;
		ros::ServiceServer app_manual_config_;
		ros::ServiceServer app_clean_cfg_save_;
		ros::ServiceServer app_clean_cfg_get_;
		ros::ServiceServer app_combine_plans_;
		ros::ServiceServer app_clean_record_;
		ros::ServiceServer app_record_summary_;
		ros::ServiceServer app_record_map_;
		ros::ServiceServer app_modify_map_;
		ros::ServiceClient pause_;
		ros::ServiceClient get_scrubber_config_;
		ros::ServiceClient set_manual_config_;
		ros::ServiceClient set_clean_config_;
		ros::ServiceClient get_clean_config_;
		//db_client
		ros::ServiceClient db_get_map_;
		ros::ServiceClient db_add_plan_;
		ros::ServiceClient db_add_path_;
		ros::ServiceClient db_add_zone_;
		ros::ServiceClient db_get_plan_;
		ros::ServiceClient db_get_path_;
		ros::ServiceClient db_get_zone_;
		ros::ServiceClient db_get_config_scale_;
		ros::ServiceClient db_update_plan_init_;
		ros::ServiceClient db_get_list_;
		ros::ServiceClient db_get_list_in_map_;
		ros::ServiceClient db_get_autofill_zone_;
		ros::ServiceClient db_delete_;
		ros::ServiceClient db_update_plan_;
		ros::ServiceClient db_update_map_;
		ros::ServiceClient db_paths_info_;


		std::thread                              scrubber_thread_;
		geometry_msgs::PoseWithCovarianceStamped cur_pose_;
		roborock_app::RosMapInfo                 app_map_info_;
		roborock_app::RobotStatus                app_status_;
		nav_msgs::Path                           origianl_path_;
		nav_msgs::OccupancyGrid                  original_map_;
		sensor_msgs::LaserScan                   original_laser_;

		actionlib::SimpleActionClient<launcher_msgs::LauncherAction>* launcher_;
	};
}

