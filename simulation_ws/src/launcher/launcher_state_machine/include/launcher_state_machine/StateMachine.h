
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
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <mbf_msgs/ExePathAction.h>
#include <mbf_msgs/GetPathAction.h>
#include <amcl/RectPara.h>
#include <launcher_msgs/PrepareNode.h>
#include <launcher_msgs/LauncherAction.h>
#include <scrubber_msgs/WayPoint.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <db_msgs/GetListInMap.h>
#include <db_msgs/GetInitialZones.h>
#include <db_msgs/GetZone.h>
#include <db_msgs/TypePath.h>
#include <db_msgs/TypeQuery.h>
#include <db_msgs/TypeZone.h>
#include <costmap_prohibition_layer/AddProhibitionZone.h>
#include "ScrubberStates.h"
#include "Automatic.h"
#include "Mapping.h"
#include "Tracking.h"
#include "Manual.h"

namespace rock::scrubber::launcher {
	class StateMachine {
	public:
		using LauncherActionServer = actionlib::SimpleActionServer<launcher_msgs::LauncherAction>;
		using Actions = launcher_msgs::State;

		StateMachine();

		~StateMachine() = default;

	private:
		// Callbacks
		void executeCallback(const launcher_msgs::LauncherGoalConstPtr& goal);

		void pauseCallback(const std_msgs::Bool::ConstPtr& msg);

		void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

		void simpleGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

		// Thread
		void standbyThread();

		// Services
		bool pauseService(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);

		bool resumeService(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);

		bool startService(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&);

		bool relocationService(amcl::RectParaRequest&, amcl::RectParaResponse&);

		bool prepareService(launcher_msgs::PrepareNodeRequest& req, launcher_msgs::PrepareNodeResponse& resp);

		bool gotoService(scrubber_msgs::WayPointRequest& req, scrubber_msgs::WayPointResponse&);

		bool gotoCancelService(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&);

		bool gotoResumeService(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&);

		bool prepareState(uint8_t state);

		bool relocation();

		bool addProhebitionZone();

		double quaternionToYaw(geometry_msgs::Quaternion& orientation);

		launcher_msgs::LauncherResult getResult();

		launcher_msgs::LauncherFeedback getFeedback();

		std::shared_ptr<ScrubberStates>       scrubber_state_;
		std::shared_ptr<Mission>              mission_;
		std::shared_ptr<Automatic>            automatic_;
		std::unique_ptr<LauncherActionServer> as_;

		ros::ServiceServer pause_srv_;
		ros::ServiceServer resume_srv_;
		ros::ServiceServer start_srv_;
		ros::ServiceServer prepare_srv_;
		ros::ServiceServer relocation_srv_;
		ros::ServiceServer waypoint_srv_;
		ros::ServiceServer pause_goto_;
		ros::ServiceServer resume_goto_;
		ros::Subscriber    pause_sub_;
		ros::Subscriber    pose_sub_;
		ros::Subscriber    goal_sub_;
		ros::Publisher     state_pub_;

		geometry_msgs::PoseWithCovarianceStamped cur_pose_;
		launcher_msgs::LauncherGoal              cur_goal_;
		launcher_msgs::LauncherGoal              pre_goal_;
		std::thread                              standby_;
		bool                                     pause_;
		bool                                     relocated_;
		uint8_t                                  node_state_;//Subprocess state
		FILE* np_; //Node process
	};
}//namespace launcher_state_machine

