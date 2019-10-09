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
#include <sys/prctl.h>
#include <signal.h>
#include <thread>
#include <vector>
#include <mbf_msgs/MoveBaseAction.h>
#include <mbf_msgs/ExePathAction.h>
#include <mbf_msgs/GetPathAction.h>
#include <db_msgs/GetPlan.h>
#include <db_msgs/GetTask.h>
#include <scrubber_msgs/SetPathConfig.h>
#include <scrubber_msgs/SetCleanConfig.h>
#include <actionlib/client/simple_action_client.h>
#include "Mission.h"
#include "ScrubberStates.h"

namespace rock::scrubber::launcher {
	class Automatic : public Mission {
	public:
		typedef actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> PointCtrl; //"move_base"
		typedef actionlib::SimpleActionClient<mbf_msgs::ExePathAction>  ExePathCtrl; // "exe_path"
		typedef actionlib::SimpleActionClient<mbf_msgs::GetPathAction>  GetPath; // "get_path"

		Automatic(std::shared_ptr<ScrubberStates>& states, uint32_t mission_id);

		~Automatic() override;

		void start() override;

		void pause() override;

		void resume() override;

		void cancel() override { cancelled_ = true; }

		bool isDone() const override { return done_; }

		bool isSuccess() const override { return success_; }

	private:
		void exeThread();

		void moveBaseUpdate(const mbf_msgs::MoveBaseFeedbackConstPtr& curPose);

		void cancelGoal();

		bool sendGoal(mbf_msgs::MoveBaseGoal&);

		bool sendGoal(mbf_msgs::ExePathGoal&);

		void updateMission();

		bool setConfig(uint32_t config_id);

		bool resetConfig();

		uint32_t id_;

		bool active_;
		bool using_move_base_;
		bool pause_;
		bool move_base_ol_;
		bool exe_path_ol_;
		bool get_path_ol_;

		std::thread automatic_thread_;

		std::vector<uint32_t> tasks_;

		geometry_msgs::PoseStamped      cur_pose_;
		std::unique_ptr<PointCtrl>      movebase_;
		std::unique_ptr<ExePathCtrl>    path_ctrl_;
		std::unique_ptr<GetPath>        get_path_;
		std::shared_ptr<ScrubberStates> states_;
	};
}

