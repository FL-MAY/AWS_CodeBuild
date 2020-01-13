//
// Created by longyue on 19-7-31.
//

#pragma once

#include <ros/ros.h>
#include <sqlite3.h>
#include <vector>
#include <std_srvs/Empty.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <db_msgs/TypePath.h>
#include <db_msgs/TypeQuery.h>
#include <db_msgs/TypeZone.h>
#include <db_msgs/TypeConfig.h>
#include <db_msgs/ConfigMode.h>
#include <db_msgs/BindAutofill.h>
#include <db_msgs/GetAutofillPlan.h>
#include <db_msgs/GetAutofillZone.h>
#include <db_msgs/GetPlan.h>
#include <db_msgs/AddPlan.h>
#include <db_msgs/UpdatePlan.h>
#include <db_msgs/UpdateInit.h>
#include <db_msgs/GetTask.h>
#include <db_msgs/Rename.h>
#include <db_msgs/Delete.h>
#include <db_msgs/GetScrubberConfig.h>
#include <db_msgs/AddMap.h>
#include <db_msgs/GetMap.h>
#include <db_msgs/UpdateMap.h>
#include <db_msgs/GetListInMap.h>
#include <db_msgs/GetInitialZones.h>
#include <db_msgs/GetZone.h>
#include <db_msgs/AddZone.h>
#include <db_msgs/GetPath.h>
#include <db_msgs/AddPath.h>
#include <db_msgs/GetList.h>
#include <db_msgs/SetMarkPoint.h>
#include <db_msgs/GetMarkPoint.h>
#include <db_msgs/SetConfigScale.h>
#include <db_msgs/GetConfigScale.h>
#include <db_msgs/SetConfigMode.h>
#include <db_msgs/GetConfigMode.h>
#include <db_msgs/AddCleanRecord.h>
#include <db_msgs/GetCleanRecords.h>
#include <db_msgs/GetRecordSummary.h>
#include <db_msgs/GetRecordInfo.h>
#include <db_msgs/GetPathsInfo.h>
#include <db_msgs/ResetMap.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace rock::scrubber::database {
	struct Task {
		nav_msgs::Path path;
		uint32_t       type;
		uint32_t       config;
	};

	class DatabaseHelper {
	public:
		DatabaseHelper();

		~DatabaseHelper();


	private:
		void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

		void trackingCallback(const nav_msgs::Path::ConstPtr& msg){path_ = *msg;}

		bool openDatabase(const char* path);

		bool closeDatabase();

		bool getPlan(db_msgs::GetPlanRequest& req, db_msgs::GetPlanResponse& resp);

		bool getTask(db_msgs::GetTaskRequest& req, db_msgs::GetTaskResponse& resp);

		bool addPlan(db_msgs::AddPlanRequest& req, db_msgs::AddPlanResponse& resp);

		bool updatePlan(db_msgs::UpdatePlanRequest& req, db_msgs::UpdatePlanResponse& resp);

		bool updatePlanInit(db_msgs::UpdateInitRequest&, db_msgs::UpdateInitResponse&);

		bool updateMapInit(db_msgs::UpdateInitRequest&, db_msgs::UpdateInitResponse&);

		bool getScrubberConfig(db_msgs::GetScrubberConfigRequest& req, db_msgs::GetScrubberConfigResponse& resp);

		bool addMap(db_msgs::AddMapRequest& req, db_msgs::AddMapResponse&);

		bool getMap(db_msgs::GetMapRequest& req, db_msgs::GetMapResponse& resp);

		bool renameData(db_msgs::RenameRequest& req, db_msgs::RenameResponse&);

		bool deleteData(db_msgs::DeleteRequest& req, db_msgs::DeleteResponse&);

		bool updateMap(db_msgs::UpdateMapRequest& req, db_msgs::UpdateMapResponse&);

		bool getListInMap(db_msgs::GetListInMapRequest& req, db_msgs::GetListInMapResponse& resp);

		bool getInitialZones(db_msgs::GetInitialZonesRequest& req, db_msgs::GetInitialZonesResponse& resp);

		bool getZone(db_msgs::GetZoneRequest& req, db_msgs::GetZoneResponse& resp);

		bool addZone(db_msgs::AddZoneRequest& req, db_msgs::AddZoneResponse&);

		bool getPath(db_msgs::GetPathRequest& req, db_msgs::GetPathResponse& resp);

		bool getPathsInfo(db_msgs::GetPathsInfoRequest&, db_msgs::GetPathsInfoResponse&);

		bool addPath(db_msgs::AddPathRequest& req, db_msgs::AddPathResponse&);

		bool getList(db_msgs::GetListRequest& req, db_msgs::GetListResponse& resp);

		bool setMarkPoint(db_msgs::SetMarkPointRequest& req, db_msgs::SetMarkPointResponse& resp);

		bool getMarkPoint(db_msgs::GetMarkPointRequest& req, db_msgs::GetMarkPointResponse& resp);

		bool setConfigScale(db_msgs::SetConfigScaleRequest&, db_msgs::SetConfigScaleResponse&);

		bool getConfigScale(db_msgs::GetConfigScaleRequest&, db_msgs::GetConfigScaleResponse&);

		bool setConfigMode(db_msgs::SetConfigModeRequest&, db_msgs::SetConfigModeResponse&);

		bool getConfigMode(db_msgs::GetConfigModeRequest&, db_msgs::GetConfigModeResponse&);

		bool addCleanRecord(db_msgs::AddCleanRecordRequest&, db_msgs::AddCleanRecordResponse&);

		bool getCleanRecords(db_msgs::GetCleanRecordsRequest&, db_msgs::GetCleanRecordsResponse&);

		bool getRecordSummary(db_msgs::GetRecordSummaryRequest&, db_msgs::GetRecordSummaryResponse& resp);

		bool getRecordInfo(db_msgs::GetRecordInfoRequest&, db_msgs::GetRecordInfoResponse& resp);

		bool bindAutofill(db_msgs::BindAutofillRequest&, db_msgs::BindAutofillResponse&);

		bool getAutofillPlan(db_msgs::GetAutofillPlanRequest&, db_msgs::GetAutofillPlanResponse&);

		bool getAutofillZone(db_msgs::GetAutofillZoneRequest&, db_msgs::GetAutofillZoneResponse&);
		//Reset map
		bool resetMap(db_msgs::ResetMapRequest& req, db_msgs::ResetMapResponse& resp);
		//Clear old data
		bool clearOldPlan(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&);

		bool clearOldMap(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&);

		bool clearOldZone(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&);
		//Helper functions
		//path_id can be a sql query like "(SELECT ...)"
		bool insertPath(std::string& path_id, nav_msgs::Path& path);

		bool deletePathData(std::string& path_id);

		double quaternionToYaw(geometry_msgs::Quaternion& orientation);

		std::string getName(const std::string& type, uint32_t& id);

		uint32_t getId(const std::string& type, std::string& name);

		int32_t sqlExec(const char* sql_state, int (* callback_fuc)(void*, int, char**, char**));

		int32_t sqlExec(const char* sql_state) {
			return sqlExec(sql_state, defaultCallBack);
		}

		static int32_t listCallBack(void* handler, int element_num, char** elements, char** col_name);

		static int32_t pathCallBack(void* handler, int element_num, char** elements, char** col_name);

		static int32_t taskCallBack(void* handler, int element_num, char** elements, char** col_name);

		static int32_t defaultCallBack(void* handler, int element_num, char** elements, char** col_name);

		ros::ServiceServer rename_srv_;
		ros::ServiceServer del_srv_;
		ros::ServiceServer bind_autofill_srv_;
		ros::ServiceServer get_autofill_plan_;
		ros::ServiceServer get_autofill_zone_;
		ros::ServiceServer get_plan_srv_;
		ros::ServiceServer get_task_srv_;
		ros::ServiceServer add_plan_srv_;
		ros::ServiceServer update_plan_srv_;
		ros::ServiceServer update_plan_init_;
		ros::ServiceServer update_map_init_;
		ros::ServiceServer get_scub_config_srv_;
		ros::ServiceServer add_map_srv_;
		ros::ServiceServer get_map_srv_;
		ros::ServiceServer update_map_srv_;
		ros::ServiceServer get_map_zones_srv_;
		ros::ServiceServer get_init_zones_srv_;
		ros::ServiceServer get_zone_srv_;
		ros::ServiceServer add_zone_srv_;
		ros::ServiceServer get_path_srv_;
		ros::ServiceServer add_path_srv_;
		ros::ServiceServer get_paths_info_;
		ros::ServiceServer get_list_srv_;
		ros::ServiceServer set_mark_srv_;
		ros::ServiceServer get_mark_srv_;
		ros::ServiceServer set_config_mode_;
		ros::ServiceServer get_config_mode_;
		ros::ServiceServer set_clean_config_scale_;
		ros::ServiceServer get_clean_config_scale_;
		ros::ServiceServer add_clean_record_;
		ros::ServiceServer get_clean_records_;
		ros::ServiceServer get_record_info_;
		ros::ServiceServer get_record_summary_;
		ros::ServiceServer reset_map_;
		ros::ServiceServer clear_map_srv_;
		ros::ServiceServer clear_plan_srv_;
		ros::ServiceServer clear_zone_srv_;
		ros::Subscriber    map_sub_;
		ros::Subscriber    path_sub_;
		ros::Publisher     map_pub_;
		ros::Publisher     map_id_pub_;

		char* err_msg_;
		bool                  is_open_;
		nav_msgs::Path        path_;
		std::vector<uint32_t> tasks_;
		std::vector<uint32_t> zones_;
		Task                  task_content_;
		sqlite3     * sql_db_;
		sqlite3_stmt* stmt_;
		nav_msgs::OccupancyGrid map_;
	};
}


