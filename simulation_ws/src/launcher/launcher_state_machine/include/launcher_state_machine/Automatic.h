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
#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <dynamic_reconfigure/client.h>
//#include <dwa_local_planner/DWAPlannerConfig.h>
//#include <teb_local_planner/TebLocalPlannerReconfigureConfig.h>
#include <costmap_2d/InflationPluginConfig.h>
#include <sys/prctl.h>
#include <signal.h>
#include <thread>
#include <vector>
#include <mbf_msgs/MoveBaseAction.h>
#include <mbf_msgs/ExePathAction.h>
#include <mbf_msgs/GetPathAction.h>
#include <db_msgs/GetPlan.h>
#include <db_msgs/GetTask.h>
#include <db_msgs/GetListInMap.h>
#include <db_msgs/Task.h>
#include <db_msgs/TypePath.h>
#include <db_msgs/TypeQuery.h>
#include <db_msgs/TypeZone.h>
#include <db_msgs/AddPath.h>
#include <scrubber_msgs/SetPathConfig.h>
#include <scrubber_msgs/SetCleanConfig.h>
#include <scrubber_msgs/CrossBump.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Empty.h>
#include "Mission.h"
#include "ScrubberStates.h"
#include "Report.h"

namespace rock::scrubber::launcher {
	class Automatic : public Mission {
	public:
		typedef actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> PointCtrl; //"move_base"
		typedef actionlib::SimpleActionClient<mbf_msgs::ExePathAction>  ExePathCtrl; // "exe_path"
		typedef actionlib::SimpleActionClient<mbf_msgs::GetPathAction>  GetPath; // "get_path"

		Automatic(std::shared_ptr<ScrubberStates>& states, int32_t mission_id, uint8_t config, int32_t exe_times = 1);

		~Automatic() override;

		void start() override;

		void pause() override;

		void pause(bool stop_clean);

		void resume() override;

		void cancel() override { cancelled_ = true; }

		void setWayPoint(geometry_msgs::PoseStamped& target_pose);

		void cancelGoto();

		void resumeGoto();

		bool isDone() const override { return done_ && tasks_.empty() && !active_; }

		bool isSuccess() const override { return success_; }

		bool goingTo() { return in_goto_; }

	private:
		void exeThread();

		void gotoThread();

		void obstacleAvoid();

		void moveBaseUpdate(const mbf_msgs::MoveBaseFeedbackConstPtr& curPose);

		void cancelGoal();

		void updateMission();

		void updateStatistics();

		void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

		void initReport(std::vector<db_msgs::Task>& task);

		bool sendGoal(mbf_msgs::MoveBaseGoal&);

		bool sendGoal(mbf_msgs::ExePathGoal&);

		bool setConfig(uint32_t config_id);

		int indexOfPose(nav_msgs::Path& path, geometry_msgs::PoseStamped& pose, int start_index);

		bool collisionAhead(nav_msgs::Path& path, int start_index, int& next_free_pose);

		bool isFree(geometry_msgs::PoseStamped& pose);

		bool resetConfig();

		void allowBackward(bool allow = true);

		double squareDistance(geometry_msgs::PoseStamped& p1, geometry_msgs::PoseStamped& p2);

		uint32_t                   id_;
		uint8_t                    retry_;
		uint8_t                    saved_state_;
		uint8_t                    config_id_;
		tf2_ros::Buffer            tf_buffer_;
		tf2_ros::TransformListener tf2_;

		bool saved_controller_;
		bool active_;
		bool in_goto_;
		bool new_goal_;
		bool new_path_;
		bool cross_bump_;
		bool using_move_base_;
		bool resume_from_goto_;
		bool pause_;
		bool goto_pause_;
		bool goto_cancel_;
		bool move_base_ol_;
		bool exe_path_ol_;
		bool get_path_ol_;
		//bool dwa_config_ol_;
		//bool teb_config_ol_;
		bool inf_config_ol_;
		bool enable_avoidance_;

		std::thread automatic_thread_;
		std::thread goto_thread_;
		std::thread obstacle_avoid_;

		std::vector<db_msgs::Task> tasks_;
		ros::Subscriber            pose_sub_;
		ros::Publisher             path_pub_;
		ros::Publisher             allow_backward_;
		nav_msgs::Path             path_;
		db_msgs::GetTask           task_srv_;

		nav_msgs::Path& cur_route_ = task_srv_.response.path;

		mbf_msgs::MoveBaseGoal       target_pose_;
		geometry_msgs::PoseStamped   cur_pose_;
		geometry_msgs::PoseStamped   resume_pose_;
		std::unique_ptr<PointCtrl>   movebase_;
		std::unique_ptr<ExePathCtrl> path_ctrl_;
		std::unique_ptr<GetPath>     get_path_;
		std::unique_ptr<Report>      report_;

		std::unique_ptr<dynamic_reconfigure::Client<costmap_2d::InflationPluginConfig>>                   inf_client_;
		//std::unique_ptr<dynamic_reconfigure::Client<dwa_local_planner::DWAPlannerConfig>>                 dwa_client_;
		//std::unique_ptr<dynamic_reconfigure::Client<teb_local_planner::TebLocalPlannerReconfigureConfig>> teb_client_;
		std::shared_ptr<ScrubberStates>                                                                   states_;
		std::shared_ptr<costmap_2d::Costmap2DROS>                                                         costmap_;
		//dwa_local_planner::DWAPlannerConfig                                                               dwa_config_;
		//teb_local_planner::TebLocalPlannerReconfigureConfig                                               teb_config_;
		costmap_2d::InflationPluginConfig                                                                 inf_config_;

	};
}

