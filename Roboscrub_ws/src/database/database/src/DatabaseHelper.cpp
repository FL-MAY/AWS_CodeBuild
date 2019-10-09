//
// Created by longyue on 19-7-31.
//

#include "database/DatabaseHelper.h"

namespace rock::scrubber::database {
	DatabaseHelper::DatabaseHelper() : err_msg_(nullptr), is_open_(false) {
		//make this path param
		ros::NodeHandle nh("~");
		ros::NodeHandle n;
		std::string     db_path;
		nh.param("db_path", db_path,
		         std::string("/home/longyue/git_repo/RoboScrub_Nav/src/database/database/db/test1.db"));

		if (!openDatabase(db_path.c_str())) {
			ROS_ERROR("Can not open database");
		}

		tasks_               = {};
		task_content_        = {};
		map_sub_             = n.subscribe("map", 1, &DatabaseHelper::mapCallback, this);
		path_sub_            = n.subscribe("launcher/tracking_path", 1, &DatabaseHelper::trackingCallback, this);
		map_pub_             = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
		map_id_pub_          = nh.advertise<std_msgs::UInt32>("map_id", 1, true);
		rename_srv_          = nh.advertiseService("rename", &DatabaseHelper::renameData, this);
		del_srv_             = nh.advertiseService("delete", &DatabaseHelper::deleteData, this);
		get_plan_srv_        = nh.advertiseService("getPlan", &DatabaseHelper::getPlan, this);
		get_task_srv_        = nh.advertiseService("getTask", &DatabaseHelper::getTask, this);
		add_plan_srv_        = nh.advertiseService("addPlan", &DatabaseHelper::addPlan, this);
		update_plan_srv_     = nh.advertiseService("updatePlan", &DatabaseHelper::updatePlan, this);
		clear_plan_srv_      = nh.advertiseService("clearOldPlan", &DatabaseHelper::clearOldPlan, this);
		get_scub_config_srv_ = nh.advertiseService("getScrubberConfig", &DatabaseHelper::getScrubberConfig, this);
		add_map_srv_         = nh.advertiseService("addMap", &DatabaseHelper::addMap, this);
		get_map_srv_         = nh.advertiseService("getMap", &DatabaseHelper::getMap, this);
		update_map_srv_      = nh.advertiseService("updateMap", &DatabaseHelper::updateMap, this);
		clear_map_srv_       = nh.advertiseService("clearOldMaps", &DatabaseHelper::clearOldMap, this);
		get_map_zones_srv_   = nh.advertiseService("getListInMap", &DatabaseHelper::getListInMap, this);
		get_init_zones_srv_  = nh.advertiseService("getInitZones", &DatabaseHelper::getInitialZones, this);
		get_zone_srv_        = nh.advertiseService("getZone", &DatabaseHelper::getZone, this);
		add_zone_srv_        = nh.advertiseService("addZone", &DatabaseHelper::addZone, this);
		get_path_srv_        = nh.advertiseService("getPath", &DatabaseHelper::getPath, this);
		add_path_srv_        = nh.advertiseService("addPath", &DatabaseHelper::addPath, this);
		get_list_srv_        = nh.advertiseService("getList", &DatabaseHelper::getList, this);
		set_mark_srv_        = nh.advertiseService("setMarkPoint", &DatabaseHelper::setMarkPoint, this);
		get_mark_srv_        = nh.advertiseService("getMarkPoint", &DatabaseHelper::getMarkPoint, this);
	}

	DatabaseHelper::~DatabaseHelper() {
		sqlite3_finalize(stmt_);
		if (!closeDatabase()) {
			ROS_ERROR("Database not close properly");
		}

		delete err_msg_;
	}

	bool DatabaseHelper::openDatabase(const char* path) {
		if (SQLITE_OK != sqlite3_open(path, &sql_db_)) {
			ROS_ERROR("Open database error: %s", sqlite3_errmsg(sql_db_));
			sqlite3_close(sql_db_);
			return false;
		}

		is_open_ = true;
		return true;
	}

	bool DatabaseHelper::closeDatabase() {
		if (!is_open_) return false;

		if (SQLITE_OK != sqlite3_close(sql_db_)) {
			ROS_ERROR("Error in close database: %s", sqlite3_errmsg(sql_db_));
			return false;
		}

		return true;
	}

	int32_t DatabaseHelper::sqlExec(const char* sql_state, int (* callback_fuc)(void*, int, char**, char**)) {
		return sqlite3_exec(sql_db_, sql_state, callback_fuc, (void*) this, &err_msg_);
	}

	int32_t DatabaseHelper::listCallBack(void* handler, int element_num, char** elements, char** col_name) {
		if (!handler) {
			return -1;
		}

		if (element_num != 1) { // Should just got task_id  or zone_id only
			ROS_ERROR("Plan callback got unexpected data");
			return -1;
		}

		auto dp = (DatabaseHelper*) handler;
		if (strcmp(*col_name, "task_id") == 0) {
			dp->tasks_.push_back((uint32_t) strtol(*elements, nullptr, 0));
		} else if (strcmp(*col_name, "zone_id") == 0) {
			dp->zones_.push_back((uint32_t) strtol(*elements, nullptr, 0));
		} else {
			ROS_ERROR("Unknown col_name %s", *col_name);
		}

		return 0;
	}

	int32_t DatabaseHelper::pathCallBack(void* handler, int element_num, char** elements, char** col_name) {
		if (!handler) {
			return -1;
		}

		if (element_num != 3) { // Should just got x, y, yaw only
			ROS_ERROR("Path callback got unexpected data");
			return -1;
		}

		auto                       dp = (DatabaseHelper*) handler;
		geometry_msgs::PoseStamped pose;
		pose.header.stamp    = ros::Time::now();
		pose.header.frame_id = "map";

		for (int i = 0; i < element_num; i++) {
			if (strcmp(col_name[i], "x") == 0) {
				pose.pose.position.x = strtod(elements[i], nullptr);
			} else if (strcmp(col_name[i], "y") == 0) {
				pose.pose.position.y = strtod(elements[i], nullptr);
			} else if (strcmp(col_name[i], "theta") == 0) {
				tf2::Quaternion qt;
				qt.setRPY(0, 0, strtod(elements[i], nullptr));
				qt.normalize();
				pose.pose.orientation.x = qt.getX();
				pose.pose.orientation.y = qt.getY();
				pose.pose.orientation.z = qt.getZ();
				pose.pose.orientation.w = qt.getW();
			} else {
				ROS_ERROR("Unexpected col name %s in path callback", col_name[i]);
			}
		}

		dp->task_content_.path.poses.push_back(pose);
		return 0;
	}

	int32_t DatabaseHelper::taskCallBack(void* handler, int element_num, char** elements, char** col_name) {
		if (!handler) {
			return -1;
		}

		if (element_num != 2) { // Should just got config_id and type
			ROS_ERROR("Task callback got unexpected data");
			return -1;
		}

		auto dp = (DatabaseHelper*) handler;

		for (int i = 0; i < element_num; i++) {
			if (strcmp(col_name[i], "config_id") == 0) {
				dp->task_content_.config = (uint32_t) strtol(elements[i], nullptr, 0);
			} else if (strcmp(col_name[i], "type") == 0) {
				dp->task_content_.type = (uint32_t) strtol(elements[i], nullptr, 0);
			} else {
				ROS_ERROR("Unexpected col name %s in task callback", col_name[i]);
			}
		}

		return 0;
	}

	int32_t DatabaseHelper::defaultCallBack(void* handler, int element_num, char** elements, char** col_name) {
		for (int i = 0; i < element_num; i++) {
			ROS_INFO("%s = %s", col_name[i], elements[i] ? elements[i] : "NULL");
		}

		return 0;
	}

	bool DatabaseHelper::getPlan(db_msgs::GetPlanRequest& req, db_msgs::GetPlanResponse& resp) {
		ROS_INFO("Plan Requested");
		std::string sql_state;
		if (req.by_plan_name) {
			sql_state = "SELECT task_id FROM Task JOIN Plan"
			            " ON Task.plan_id = Plan.plan_id "
			            "WHERE name = '";
			sql_state += req.plan_name;
			sql_state += "' ORDER BY idx ASC;";
		} else {
			sql_state = "SELECT task_id FROM Task JOIN Plan"
			            " ON Task.plan_id = Plan.plan_id "
			            "WHERE Plan.plan_id = ";
			sql_state += std::to_string(req.plan_id) + " ORDER BY idx ASC;";
		}

		if (SQLITE_OK == sqlExec(sql_state.c_str(), listCallBack)) {
			resp.tasks = tasks_;
			tasks_.clear();// Clear task vector
			return true;
		}

		ROS_ERROR("%s", err_msg_);
		tasks_.clear();// Clear task vector
		return false;
	}

	bool DatabaseHelper::getTask(db_msgs::GetTaskRequest& req, db_msgs::GetTaskResponse& resp) {
		ROS_INFO("Task Requested");
		std::string path_state = "SELECT x, y, theta FROM PathData as P JOIN FollowPath as FP "
		                         "ON P.path_id = FP.path_id WHERE task_id = ";
		path_state += std::to_string(req.task_id);
		path_state += " ORDER BY idx asc; ";

		std::string task_state = "SELECT config_id, type FROM Task Where task_id = ";
		task_state += std::to_string(req.task_id);
		task_state += ";";

		task_content_.path.header.frame_id = "map";
		task_content_.path.header.stamp    = ros::Time::now();

		if (SQLITE_OK == sqlExec(path_state.c_str(), pathCallBack)
		    && SQLITE_OK == sqlExec(task_state.c_str(), taskCallBack)) {
			resp.path      = task_content_.path;
			resp.config_id = task_content_.config;
			resp.type      = task_content_.type;
			task_content_ = {}; //clear task
			return true;
		}

		ROS_ERROR("%s", err_msg_);
		task_content_ = {}; //clear task
		return false;
	}

	bool DatabaseHelper::addPlan(db_msgs::AddPlanRequest& req, db_msgs::AddPlanResponse& resp) {
		if (SQLITE_OK != sqlExec("BEGIN TRANSACTION")) {
			ROS_ERROR("%s", err_msg_);
			return false;
		}

		//Adding a plan
		std::string state = "INSERT INTO PLAN(name, map_id) VALUES('" + req.name + "' "
		                    + std::to_string(req.map_id) + ");";
		if (SQLITE_OK != sqlExec(state.c_str())) {
			sqlExec("ROLLBACK");
			ROS_ERROR("%s", err_msg_);
			return false;
		}

		//Binding tasks
		for (int i = 0; i < req.ordered_tasks.size(); i++) {
			state = "INSERT INTO Task(plan_id, config_id, type, idx) VALUES(";
			state += "(SELECT plan_id from Plan WHERE name = '" + req.name + "'),";
			state += std::to_string(req.ordered_tasks[i].config_id) + ",";
			state += std::to_string(req.ordered_tasks[i].type) + ",";
			state += std::to_string(i) + ");";
			if (SQLITE_OK != sqlExec(state.c_str())) { //Insert task;
				ROS_ERROR("%s", err_msg_);
				sqlExec("ROLLBACK");
				return false;
			}

			state = "INSERT INTO FollowPath VALUES((SELECT max(task_id) from Task), (SELECT path_id from Path ";
			state += "WHERE name = '" + req.ordered_tasks[i].path_name + "'));";
			if (SQLITE_OK != sqlExec(state.c_str())) { // Binding task and path;
				ROS_ERROR("%s", err_msg_);
				sqlExec("ROLLBACK");
				return false;
			}
		}

		if (SQLITE_OK != sqlExec("END TRANSACTION")) {
			ROS_ERROR("%s", err_msg_);
			sqlExec("ROLLBACK");
			return false;
		}

		resp.plan_id = getId(db_msgs::Type::PLAN, req.name);
		return true;
	}

	bool DatabaseHelper::updatePlan(db_msgs::UpdatePlanRequest& req, db_msgs::UpdatePlanResponse& resp) {
		//Adding plan using a template name
		std::string              new_name = req.plan_name + "_new_";
		db_msgs::AddPlanRequest  add_plan_req;
		db_msgs::AddPlanResponse add_plan_resp;
		add_plan_req.name          = new_name;
		add_plan_req.ordered_tasks = req.ordered_tasks;
		if (!addPlan(add_plan_req, add_plan_resp)) {
			ROS_ERROR("%s", err_msg_);
			return false;
		}

		//Rename old one
		db_msgs::RenameRequest  re_req;
		db_msgs::RenameResponse re_resp;
		re_req.type                        = db_msgs::Type::PLAN;
		re_req.old_name                    = req.plan_name;
		re_req.new_name                    = req.plan_name + "_" + std::to_string(ros::Time::now().sec);

		if (!renameData(re_req, re_resp)) {
			ROS_ERROR("Unable to save old plan name %s, plan name now is %s", req.plan_name.c_str(), new_name.c_str());
			ROS_ERROR("%s", err_msg_);
			return false;
		}
		//Set old flag
		std::string             flag_state = "UPDATE Plan SET old = 1 WHERE name = '" + re_req.new_name + "';";
		if (SQLITE_OK != sqlExec(flag_state.c_str())) {
			ROS_ERROR("Unable to set old flag");
			ROS_ERROR("%s", err_msg_);
			return false;
		}
		//Rename new plan to original name
		re_req.old_name = new_name;
		re_req.new_name = req.plan_name;
		if (!renameData(re_req, re_resp)) {
			ROS_ERROR("Unable to use plan name %s, plan name now is %s", req.plan_name.c_str(), new_name.c_str());
			ROS_ERROR("%s", err_msg_);
			return false;
		}

		return true;
	}

	bool DatabaseHelper::clearOldPlan(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&) {
		if (SQLITE_OK != sqlExec("BEGIN TRANSACTION")) {
			ROS_ERROR("%s", err_msg_);
			return false;
		}

		//Deleting path binding
		std::string plan_id = "(SELECT plan_id FROM Plan WHERE old != 0)";
		std::string state   = "DELETE FROM FollowPath WHERE task_id in "
		                      "(SELECT task_id from Task WHERE plan_id in " + plan_id + ");";
		if (SQLITE_OK != sqlExec(state.c_str())) {
			ROS_ERROR("%s", err_msg_);
			sqlExec("ROLLBACK");
			return false;
		}

		//Deleting tasks
		state = "DELETE FROM Task WHERE plan_id in " + plan_id + ";";
		if (SQLITE_OK != sqlExec(state.c_str())) {
			ROS_ERROR("%s", err_msg_);
			sqlExec("ROLLBACK");
			return false;
		}

		//Deleting plan itself
		state = "DELETE FROM Plan WHERE old != 0;";
		if (SQLITE_OK != sqlExec(state.c_str())) {
			ROS_ERROR("%s", err_msg_);
			sqlExec("ROLLBACK");
			return false;
		}

		if (SQLITE_OK != sqlExec("END TRANSACTION")) {
			ROS_ERROR("%s", err_msg_);
			sqlExec("ROLLBACK");
			return false;
		}

		return true;
	}

	bool DatabaseHelper::getScrubberConfig(db_msgs::GetScrubberConfigRequest& req,
	                                       db_msgs::GetScrubberConfigResponse& resp) {
		std::string state = "SELECT squeegee FROM ScrubberConfig where config_id = "
		                    + std::to_string(req.config_id) + ";";
		if (SQLITE_OK != sqlite3_prepare_v2(sql_db_, state.c_str(), state.length(), &stmt_, nullptr)) {
			ROS_ERROR("Error in get squeegee prepare: %s", sqlite3_errmsg(sql_db_));
			return false;
		}

		sqlite3_step(stmt_);
		resp.squeegee = sqlite3_column_int(stmt_, 0);

		state = "SELECT value FROM ConfigValue where value_id = (SELECT brush from ScrubberConfig "
		        "WHERE config_id = " + std::to_string(req.config_id) + ");";
		if (SQLITE_OK != sqlite3_prepare_v2(sql_db_, state.c_str(), state.length(), &stmt_, nullptr)) {
			ROS_ERROR("Error in get brush prepare: %s", sqlite3_errmsg(sql_db_));
			return false;
		}

		sqlite3_step(stmt_);
		resp.brush = sqlite3_column_int(stmt_, 0);

		state = "SELECT value FROM ConfigValue where value_id = (SELECT flow from ScrubberConfig "
		        "WHERE config_id = " + std::to_string(req.config_id) + ");";
		if (SQLITE_OK != sqlite3_prepare_v2(sql_db_, state.c_str(), state.length(), &stmt_, nullptr)) {
			ROS_ERROR("Error in get flow prepare: %s", sqlite3_errmsg(sql_db_));
			return false;
		}

		sqlite3_step(stmt_);
		resp.flow = sqlite3_column_int(stmt_, 0);

		state = "SELECT value FROM ConfigValue where value_id = (SELECT vacuum from ScrubberConfig "
		        "WHERE config_id = " + std::to_string(req.config_id) + ");";
		if (SQLITE_OK != sqlite3_prepare_v2(sql_db_, state.c_str(), state.length(), &stmt_, nullptr)) {
			ROS_ERROR("Error in get vacuum prepare: %s", sqlite3_errmsg(sql_db_));
			return false;
		}

		sqlite3_step(stmt_);
		resp.vacuum = sqlite3_column_int(stmt_, 0);

		sqlite3_reset(stmt_);
		return true;
	}

	void DatabaseHelper::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
		map_.data   = msg->data;
		map_.header = msg->header;
		map_.info   = msg->info;
	}

	bool DatabaseHelper::addMap(db_msgs::AddMapRequest& req, db_msgs::AddMapResponse& resp) {
		if (map_.data.empty()) {
			ROS_ERROR("Currently no map");
			return false;
		}

		stmt_ = nullptr;
		if (SQLITE_OK != sqlExec("BEGIN TRANSACTION")) {
			ROS_ERROR("%s", err_msg_);
			return false;
		}

		std::string state = "INSERT INTO Map(name, create_time) VALUES('" + req.name + "',"
		                    + std::to_string(ros::Time::now().toSec()) + ");";

		if (SQLITE_OK != sqlite3_prepare_v2(sql_db_, state.c_str(), state.length(), &stmt_, nullptr)) {
			ROS_ERROR("ERROR IN PREPARE: %s", sqlite3_errmsg(sql_db_));
			sqlExec("ROLLBACK");
			return false;
		}

		sqlite3_step(stmt_);

		double      left   = map_.info.origin.position.y + map_.info.resolution * map_.info.width / 2;
		double      top    = map_.info.origin.position.x + map_.info.resolution * map_.info.height / 2;
		std::string map_id = "(SELECT map_id FROM Map where name = '" + req.name + "')";
		state = "INSERT INTO MapData (map_id, modify_time, resolution, width, height, left, top, data) VALUES ("
		        + map_id +
		        "," + std::to_string(ros::Time::now().toSec()) + "," + std::to_string(map_.info.resolution) + ","
		        + std::to_string(map_.info.width) + "," + std::to_string(map_.info.height) + ","
		        + std::to_string(left) + "," + std::to_string(top) + ", ?);";

		int32_t size = sizeof(map_.data[0]) * map_.data.size();
		ROS_INFO("Map size %d byte", size);

		if (SQLITE_OK != sqlite3_prepare_v2(sql_db_, state.c_str(), state.length(), &stmt_, nullptr)) {
			ROS_ERROR("ERROR IN PREPARE: %s", sqlite3_errmsg(sql_db_));
			sqlExec("ROLLBACK");
			return false;
		}


		if (SQLITE_OK != sqlite3_bind_blob(stmt_, 1, map_.data.data(), size, SQLITE_STATIC)) {
			ROS_ERROR("ERROR IN BIND: %s", sqlite3_errmsg(sql_db_));
			sqlExec("ROLLBACK");
			return false;
		}

		/*//For saving pbstream map
		FILE* pf = nullptr;//fopen("/home/longyue/map.pbstream","rb");
		if(pf){
			fseek(pf, 0, SEEK_END);
			int32_t pbs_size = ftell(pf);
			rewind(pf);
			auto buffer = (char*) malloc(sizeof(char)*pbs_size);
			fread(buffer,1, pbs_size,pf);
			int32_t f_size = sizeof(char)*pbs_size;
			ROS_INFO("pbstrem size %d",f_size);
			if (SQLITE_OK != sqlite3_bind_blob(stmt_, 2, buffer, f_size, SQLITE_STATIC)) {
				ROS_ERROR("ERROR IN BIND: %s", sqlite3_errmsg(sql_db_));
				sqlExec("ROLLBACK");
				fclose(pf);
				free(buffer);
				return false;
			}
			fclose(pf);
			free(buffer);
		}
		*/
		sqlite3_step(stmt_);

		sqlite3_reset(stmt_);
		if (SQLITE_OK != sqlExec("END TRANSACTION")) {
			ROS_ERROR("%s", err_msg_);
			sqlExec("ROLLBACK");
			return false;
		}

		resp.map_id = getId(db_msgs::Type::MAP, req.name);
		//Add initial zone
		db_msgs::AddPath           add_path;
		db_msgs::AddZone           add_zone;
		geometry_msgs::PoseStamped pose;
		add_path.request.by_topic = false;
		add_path.request.name     = req.name + "_Init_Vertex";
		pose.pose.orientation.w   = 1;
		pose.pose.position.x      = 0.5;
		pose.pose.position.y      = 0.5;
		add_path.request.path.poses.push_back(pose);
		pose.pose.position.x = 0.5;
		pose.pose.position.y = -0.5;
		add_path.request.path.poses.push_back(pose);
		pose.pose.position.x = -0.5;
		pose.pose.position.y = -0.5;
		add_path.request.path.poses.push_back(pose);
		pose.pose.position.x = -0.5;
		pose.pose.position.y = 0.5;
		add_path.request.path.poses.push_back(pose);
		if (!addPath(add_path.request, add_path.response)) {
			ROS_ERROR("Unable to setup initial zones at (0,0)");
		}

		add_zone.request.name      = req.name + "_Init_Zone";
		add_zone.request.map_id    = resp.map_id;
		add_zone.request.path_id   = add_path.response.path_id;
		add_zone.request.config_id = 1; //All off
		add_zone.request.type      = db_msgs::Type::INIT;
		if (!addZone(add_zone.request, add_zone.response)) {
			ROS_ERROR("Error set up initial zone for map %d", resp.map_id);
		}

		return true;
	}

	bool DatabaseHelper::getMap(db_msgs::GetMapRequest& req, db_msgs::GetMapResponse& resp) {
		std::string map_id = req.by_map_name ? "SELECT map_id FROM Map WHERE name = '" + req.map_name + "'"
		                                     : std::to_string(req.map_id);
		std::string state  = "SELECT resolution, width, height, left, top, data FROM MapData WHERE map_id = (" + map_id
		                     + ");";
		if (req.by_map_name) {
			if (SQLITE_OK != sqlite3_prepare_v2(sql_db_, map_id.c_str(), map_id.length(), &stmt_, nullptr)) {
				ROS_ERROR("Error in get map_id prepare: %s", sqlite3_errmsg(sql_db_));
				return false;
			}
			sqlite3_step(stmt_);
			resp.map_id = sqlite3_column_int(stmt_, 0);

		} else {
			resp.map_id = req.map_id;
		}

		if (SQLITE_OK != sqlite3_prepare_v2(sql_db_, state.c_str(), state.length(), &stmt_, nullptr)) {
			ROS_ERROR("Error in get map prepare: %s", sqlite3_errmsg(sql_db_));
			return false;
		}

		sqlite3_step(stmt_);

		resp.map.header.stamp              = ros::Time::now();
		resp.map.header.frame_id           = "map";
		resp.map.info.resolution           = sqlite3_column_double(stmt_, 0);
		resp.map.info.width                = sqlite3_column_int(stmt_, 1);
		resp.map.info.height               = sqlite3_column_int(stmt_, 2);
		resp.map.info.origin.position.y    = sqlite3_column_double(stmt_, 3)
		                                     - resp.map.info.resolution * resp.map.info.width / 2;
		resp.map.info.origin.position.x    = sqlite3_column_double(stmt_, 4)
		                                     - resp.map.info.resolution * resp.map.info.height / 2;
		resp.map.info.origin.orientation.w = 1;

		uint32_t size     = resp.map.info.width * resp.map.info.height;
		auto     raw_data = (const int8_t*) sqlite3_column_blob(stmt_, 5);

		resp.map.data = {raw_data, raw_data + size};

		if (req.pub_this_map) {
			nav_msgs::OccupancyGrid nav_map = resp.map;

			// Adjust occupied value
			// Original data saved from mapper were -1 ~ 100
			// -1 unknown, 0~25 free, 65~100 occupied, others unknown

			for (int8_t& x : nav_map.data) {
				if (x > 0) {
					if (x <= 25) { //Free
						x = 0; //Free for cost map
					} else if (x >= 65) { //Occupied
						x = 100;
					} else { //Unknown
						x = -1;
					}
				}
			}

			std_msgs::UInt32 cur_map_id;
			cur_map_id.data = resp.map_id;
			map_pub_.publish(nav_map);
			map_id_pub_.publish(cur_map_id);
		}

		sqlite3_reset(stmt_);
		return true;
	}

	bool DatabaseHelper::renameData(db_msgs::RenameRequest& req, db_msgs::RenameResponse&) {
		std::string state = "UPDATE " + req.type + " SET name = '" + req.new_name + "' WHERE name = '" + req.old_name
		                    + "';";
		if (SQLITE_OK != sqlExec(state.c_str())) {
			ROS_ERROR("Error in rename %s: %s", req.type.c_str(), sqlite3_errmsg(sql_db_));
			return false;
		}

		return true;
	}

	bool DatabaseHelper::deleteData(db_msgs::DeleteRequest& req, db_msgs::DeleteResponse&) {
		if (SQLITE_OK != sqlExec("BEGIN TRANSACTION")) {
			ROS_ERROR("%s", err_msg_);
			return false;
		}

		db_msgs::Rename rename;
		rename.request.type     = req.type;
		rename.request.old_name = req.name;
		rename.request.new_name = req.name + "_" + std::to_string(ros::Time::now().sec);
		if (!renameData(rename.request, rename.response)) {
			sqlExec("ROLLBACK");
			return false;
		}

		std::string state = "UPDATE " + req.type + " SET old = 1 WHERE name = '" + rename.request.new_name + "';";
		if (SQLITE_OK != sqlExec(state.c_str())) {
			ROS_ERROR("Error in set %s old %s", req.type.c_str(), err_msg_);
			sqlExec("ROLLBACK");
			return false;
		}

		if (SQLITE_OK != sqlExec("END TRANSACTION")) {
			ROS_ERROR("%s", err_msg_);
			sqlExec("ROLLBACK");
			return false;
		}

		return true;
	}

	bool DatabaseHelper::updateMap(db_msgs::UpdateMapRequest& req, db_msgs::UpdateMapResponse& resp) {
		//Adding current map as temp
		std::string     temp_name = req.name + "_temp_";
		db_msgs::AddMap temp_map;
		temp_map.request.name = temp_name;
		if (!addMap(temp_map.request, temp_map.response)) {
			ROS_INFO("Update map Failed in creating temp map");
			return false;
		}

		//Keep create time
		db_msgs::Delete del;
		std::string     state = "UPDATE Map SET create_time = (SELECT create_time from Map WHERE name = '"
		                        + req.name + "') WHERE name = '" + temp_name + "';";
		del.request.type = db_msgs::Type::MAP;
		if (SQLITE_OK != sqlExec(state.c_str())) {
			ROS_ERROR("%s", err_msg_);
			del.request.name = temp_name;
			deleteData(del.request, del.response);
			return false;
		}

		//Soft 'delete' old one
		del.request.name = req.name;
		if (!deleteData(del.request, del.response)) {
			ROS_ERROR("Update map Fail in delete old map");
			del.request.name = temp_name;
			deleteData(del.request, del.response);
			return false;
		}

		//Get rid of _temp_
		db_msgs::Rename rename;
		rename.request.type     = db_msgs::Type::MAP;
		rename.request.old_name = temp_name;
		rename.request.new_name = req.name;
		if (!renameData(rename.request, rename.response)) {
			ROS_ERROR("Update map Fail in rename temp");
			return false;
		}

		return true;
	}

	bool DatabaseHelper::clearOldMap(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&) {
		if (SQLITE_OK != sqlExec("BEGIN TRANSACTION")) {
			ROS_ERROR("%s", err_msg_);
			return false;
		}

		if (SQLITE_OK != sqlExec("DELETE FROM MapData WHERE map_id in (SELECT map_id FROM Map where old != 0);")) {
			ROS_ERROR("%s", err_msg_);
			sqlExec("ROLLBACK");
			return false;
		}

		if (SQLITE_OK != sqlExec("DELETE FROM Map WHERE old != 0;")) {
			ROS_ERROR("%s", err_msg_);
			sqlExec("ROLLBACK");
			return false;
		}

		if (SQLITE_OK != sqlExec("END TRANSACTION")) {
			ROS_ERROR("%s", err_msg_);
			sqlExec("ROLLBACK");
			return false;
		}

		return true;
	}

	bool DatabaseHelper::getListInMap(db_msgs::GetListInMapRequest& req, db_msgs::GetListInMapResponse& resp) {
		std::string map_id = req.by_map_name ? "SELECT map_id FROM Map WHERE name = '" + req.name + "'" :
		                     std::to_string(req.map_id);
		std::string state  = "SELECT " + req.type + "_id, name from " + req.type + " WHERE map_id = (" + map_id + ")"
		                     + " and old=0;";
		if (SQLITE_OK != sqlite3_prepare_v2(sql_db_, state.c_str(), state.length(), &stmt_, nullptr)) {
			ROS_ERROR("Error in get list prepare: %s", sqlite3_errmsg(sql_db_));
			return false;
		}

		uint32_t         result;
		db_msgs::Content content;
		while (ros::ok()) {
			result = sqlite3_step(stmt_);
			//Got all rows
			if (result == SQLITE_DONE) break;
			//Got a valid row
			if (result == SQLITE_ROW) {
				content.id   = sqlite3_column_int(stmt_, 0);
				content.name = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt_, 1)));
				resp.contents.push_back(content);
			} else {
				ROS_ERROR("Error in get list step %s", err_msg_);
				return false;
			}
		}

		return true;
	}

	bool DatabaseHelper::getZone(db_msgs::GetZoneRequest& req, db_msgs::GetZoneResponse& resp) {
		std::string zone_id = req.by_zone_name ? "SELECT zone_id from Zone WHERE name = '" + req.zone_name + "'" :
		                      std::to_string(req.zone_id);

		std::string state = "SELECT map_id, path_id, type, config_id FROM Zone JOIN ZoneConfig "
		                    "ON Zone.zone_id = ZoneConfig.zone_id WHERE Zone.zone_id = (" + zone_id + ");";

		if (SQLITE_OK != sqlite3_prepare_v2(sql_db_, state.c_str(), state.length(), &stmt_, nullptr)) {
			ROS_ERROR("Error in get map_id prepare: %s", sqlite3_errmsg(sql_db_));
			return false;
		}

		sqlite3_step(stmt_);
		db_msgs::GetPath path;
		resp.map_id          = sqlite3_column_int(stmt_, 0);
		path.request.path_id = sqlite3_column_int(stmt_, 1);
		resp.type            = sqlite3_column_int(stmt_, 2);
		resp.config_id       = sqlite3_column_int(stmt_, 3);

		getPath(path.request, path.response);
		resp.path = path.response.path;

		return true;
	}

	bool DatabaseHelper::getInitialZones(db_msgs::GetInitialZonesRequest& req, db_msgs::GetInitialZonesResponse& resp) {
		std::string state = "SELECT zone_id, name FROM Zone WHERE type=" + std::to_string(db_msgs::Type::INIT)
		                    + " AND old = 0 AND map_id="
		                    + std::to_string(req.map_id) + ";";

		if (SQLITE_OK != sqlite3_prepare_v2(sql_db_, state.c_str(), state.length(), &stmt_, nullptr)) {
			ROS_ERROR("Error in get initial Zone prepare: %s", sqlite3_errmsg(sql_db_));
			return false;
		}

		uint32_t         result;
		db_msgs::Content content;
		while (ros::ok()) {
			result = sqlite3_step(stmt_);
			//Got all rows
			if (result == SQLITE_DONE) break;
			//Got a valid row
			if (result == SQLITE_ROW) {
				content.id   = sqlite3_column_int(stmt_, 0);
				content.name = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt_, 1)));
				resp.zones.push_back(content);
			} else {
				ROS_ERROR("Error in get initial zone step %s", err_msg_);
				return false;
			}
		}

		return true;
	}

	bool DatabaseHelper::addZone(db_msgs::AddZoneRequest& req, db_msgs::AddZoneResponse& resp) {
		if (SQLITE_OK != sqlExec("BEGIN TRANSACTION")) {
			ROS_ERROR("%s", err_msg_);
			return false;
		}

		std::string zone_id = "(SELECT zone_id FROM Zone WHERE name = '" + req.name + "')";
		std::string state   = "INSERT INTO Zone (name,map_id, path_id, type) VALUES('" + req.name + "',"
		                      + std::to_string(req.map_id) + "," + std::to_string(req.path_id) + ","
		                      + std::to_string(req.type) + ");";
		if (SQLITE_OK != sqlExec(state.c_str())) {
			ROS_ERROR("%s", err_msg_);
			sqlExec("ROLLBACK");
			return false;
		}

		state = "INSERT INTO ZoneConfig VALUES (" + zone_id + "," + std::to_string(req.config_id) + ");";

		if (SQLITE_OK != sqlExec(state.c_str())) {
			ROS_ERROR("%s", err_msg_);
			sqlExec("ROLLBACK");
			return false;
		}

		if (SQLITE_OK != sqlExec("END TRANSACTION")) {
			ROS_ERROR("%s", err_msg_);
			sqlExec("ROLLBACK");
			return false;
		}

		resp.zone_id = getId(db_msgs::Type::ZONE, req.name);
		return true;
	}

	bool DatabaseHelper::getPath(db_msgs::GetPathRequest& req, db_msgs::GetPathResponse& resp) {
		std::string path_id = req.use_path_name ? "(SELECT path_id FROM Path WHERE name = '" + req.path_name + "')" :
		                      std::to_string(req.path_id);
		std::string state   = "SELECT x, y, theta FROM PathData WHERE path_id = " + path_id + " ORDER BY idx ASC";
		if (SQLITE_OK != sqlExec(state.c_str(), pathCallBack)) {
			ROS_ERROR("%s", err_msg_);
			return false;
		}

		for (int i = 0; i < task_content_.path.poses.size(); i++) {
			task_content_.path.poses[i].header.seq = i;
		}

		resp.path                 = task_content_.path;
		resp.path.header.frame_id = "map";
		resp.path.header.stamp    = ros::Time::now();
		task_content_.path        = {};

		return true;
	}

	bool DatabaseHelper::addPath(db_msgs::AddPathRequest& req, db_msgs::AddPathResponse& resp) {
		if (SQLITE_OK != sqlExec("BEGIN TRANSACTION")) {
			ROS_ERROR("%s", err_msg_);
			return false;
		}

		std::string state   = "INSERT INTO PATH (name) VALUES('" + req.name + "')";
		std::string path_id = "(SELECT path_id from Path WHERE name = '" + req.name + "')";
		if (SQLITE_OK != sqlExec(state.c_str())) {
			ROS_ERROR("%s", err_msg_);
			sqlExec("ROLLBACK");
			return false;
		}

		bool success = (req.by_topic ? insertPath(path_id, path_) : insertPath(path_id, req.path));
		if (!success) {
			sqlExec("ROLLBACK");
			return false;
		}

		if (SQLITE_OK != sqlExec("END TRANSACTION")) {
			ROS_ERROR("%s", err_msg_);
			sqlExec("ROLLBACK");
			return false;
		}

		resp.path_id = getId(db_msgs::Type::PATH, req.name);
		return true;
	}

	bool DatabaseHelper::getList(db_msgs::GetListRequest& req, db_msgs::GetListResponse& resp) {
		std::string state = "SELECT " + req.type + "_id, name FROM " + req.type + " WHERE old = 0;";
		if (SQLITE_OK != sqlite3_prepare_v2(sql_db_, state.c_str(), state.length(), &stmt_, nullptr)) {
			ROS_ERROR("Error in get list prepare: %s", sqlite3_errmsg(sql_db_));
			return false;
		}

		uint32_t         result;
		db_msgs::Content content;
		while (ros::ok()) {
			result = sqlite3_step(stmt_);
			//Got all rows
			if (result == SQLITE_DONE) break;
			//Got a valid row
			if (result == SQLITE_ROW) {
				content.id   = sqlite3_column_int(stmt_, 0);
				content.name = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt_, 1)));
				resp.contents.push_back(content);
			} else {
				ROS_ERROR("Error in get list step %s", err_msg_);
				return false;
			}
		}
		return true;
	}

	double DatabaseHelper::quaternionToYaw(geometry_msgs::Quaternion& orientation) {
		tf2::Quaternion qt(orientation.x, orientation.y, orientation.z, orientation.w);
		double          roll, pitch, yaw;
		tf2::Matrix3x3(qt).getRPY(roll, pitch, yaw);
		return yaw;
	}

	bool DatabaseHelper::insertPath(std::string& path_id, nav_msgs::Path& path) {
		if (path.poses.empty()) {
			ROS_ERROR("Try to save an empty path, nothing published on path topic yet");
			return false;
		}

		std::string state;
		double      x, y, yaw;
		for (int    i = 0; i < path.poses.size(); i++) {
			x     = path.poses[i].pose.position.x;
			y     = path.poses[i].pose.position.y;
			yaw   = quaternionToYaw(path.poses[i].pose.orientation);
			state = "INSERT INTO PathData VALUES(" + path_id + "," + std::to_string(i) + "," + std::to_string(x)
			        + "," + std::to_string(y) + "," + std::to_string(yaw) + ");";
			if (SQLITE_OK != sqlExec(state.c_str())) {
				ROS_ERROR("%s", err_msg_);
				return false;
			}
		}
		return true;
	}

	uint32_t DatabaseHelper::getId(const std::string& type, std::string& name) {
		std::string state = "SELECT " + type + "_id FROM " + type + " WHERE name = '" + name + "';";
		if (SQLITE_OK != sqlite3_prepare_v2(sql_db_, state.c_str(), state.length(), &stmt_, nullptr)) {
			ROS_ERROR("Error in get list prepare: %s", sqlite3_errmsg(sql_db_));
			return false;
		}

		if (SQLITE_ROW == sqlite3_step(stmt_)) {
			int id = sqlite3_column_int(stmt_, 0);
			return (uint32_t) id;
		}

		ROS_ERROR("Unable to get id");
		return 0;
	}

	bool DatabaseHelper::setMarkPoint(db_msgs::SetMarkPointRequest& req, db_msgs::SetMarkPointResponse& resp) {
		if (req.type < db_msgs::Type::REFILL || req.type > db_msgs::Type::PARKING) {
			ROS_ERROR("Invalid mark point type %d", req.type);
			return false;
		}

		std::string state = "SELECT path_id FROM Zone WHERE map_id = " + std::to_string(req.map_id)
		                    + " and type = " + std::to_string(req.type) + ";";
		if (SQLITE_OK != sqlite3_prepare_v2(sql_db_, state.c_str(), state.length(), &stmt_, nullptr)) {
			ROS_ERROR("Error in set mark point prepare: %s", sqlite3_errmsg(sql_db_));
			return false;
		}

		uint32_t path_id = 0;
		if (SQLITE_ROW == sqlite3_step(stmt_)) {
			path_id = sqlite3_column_int(stmt_, 0);
			state   = "UPDATE PathData SET x = " + std::to_string(req.pose.pose.position.x)
			          + ", y = " + std::to_string(req.pose.pose.position.y)
			          + ", theta = " + std::to_string(quaternionToYaw(req.pose.pose.orientation))
			          + " WHERE path_id = " + std::to_string(path_id) + " AND idx = 0;";
			if (SQLITE_OK != sqlExec(state.c_str())) {
				ROS_ERROR("%s", err_msg_);
				return false;
			}
		} else {//Mark point not set yet
			db_msgs::AddPath add_path;
			db_msgs::AddZone add_zone;
			db_msgs::Delete  del;
			add_path.request.by_topic = false;
			add_path.request.name     = "Mark_Point_" + std::to_string(req.map_id) + "_" + std::to_string(req.type);
			add_path.request.path.poses.push_back(req.pose);
			if (!addPath(add_path.request, add_path.response)) {
				ROS_ERROR("Fail to add first mark point %d", req.type);
				return false;
			}

			add_zone.request.path_id   = add_path.response.path_id;
			add_zone.request.map_id    = req.map_id;
			add_zone.request.type      = req.type;
			add_zone.request.config_id = 1;
			add_zone.request.name      = "Mark_Zone_" + std::to_string(req.map_id) + "_" + std::to_string(req.type);

			if(!addZone(add_zone.request, add_zone.response)){
				ROS_ERROR("Fail to bind first mark point %d to zone", req.type);
				del.request.name = add_path.request.name;
				del.request.type = db_msgs::Type::PATH;
				deleteData(del.request, del.response);
				return false;
			}
		}

		return true;
	}

	bool DatabaseHelper::getMarkPoint(db_msgs::GetMarkPointRequest& req, db_msgs::GetMarkPointResponse& resp) {
		if (req.type < db_msgs::Type::REFILL || req.type > db_msgs::Type::PARKING) {
			ROS_ERROR("Invalid mark point type %d", req.type);
			return false;
		}

		std::string state = "SELECT path_id FROM Zone WHERE map_id = " + std::to_string(req.map_id)
				+ " AND type = " + std::to_string(req.type)+";";

		if (SQLITE_OK != sqlite3_prepare_v2(sql_db_, state.c_str(), state.length(), &stmt_, nullptr)) {
			ROS_ERROR("Error in get mark point prepare: %s", sqlite3_errmsg(sql_db_));
			return false;
		}

		if(SQLITE_ROW != sqlite3_step(stmt_)){
			ROS_ERROR("No matching mark point");
			return false;
		}

		uint32_t path_id = sqlite3_column_int(stmt_, 0);

		db_msgs::GetPath get_path;
		get_path.request.path_id = path_id;
		get_path.request.use_path_name = false;
		if(!getPath(get_path.request, get_path.response)){
			ROS_ERROR("No mark point data");
			return false;
		}

		if(get_path.response.path.poses.empty()){
			ROS_ERROR("Got empty path");
			return false;
		}

		resp.pose = get_path.response.path.poses[0];

		return true;
	}
}