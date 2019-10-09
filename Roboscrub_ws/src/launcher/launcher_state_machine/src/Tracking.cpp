//*********************************************************
//
// Copyright (c) 2019 Roborock. All rights reserved.
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************

#include "launcher_state_machine/Tracking.h"

namespace rock::scrubber::launcher {

	Tracking::Tracking(std::shared_ptr<ScrubberStates>& states)
			: states_(states), active_(false), ready_(false) {
		ros::NodeHandle n;
		ros::NodeHandle nh("~");
		pose_sub_ = n.subscribe("amcl_pose", 5, &Tracking::poseCallback,this);
		path_pub_ = nh.advertise<nav_msgs::Path>("tracking_path",1, true);
		path_ = {};
	}

	Tracking::~Tracking() {
		if(tracking_thread_.joinable()){
			tracking_thread_.join();
		}
	}

	void Tracking::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
		if(!ready_){//Got first pose
			ready_ = true;
			states_ -> shiftState(States::TRACKING);
		}

		if(!active_) return;

		if(path_.poses.empty()){
			geometry_msgs::PoseStamped pose;
			pose.header = msg->header;
			pose.pose = msg->pose.pose;
			path_.poses.push_back(pose);
		}else{
			double dx = abs(path_.poses.back().pose.position.x - msg->pose.pose.position.x);
			double dy = abs(path_.poses.back().pose.position.y - msg->pose.pose.position.y);
			if(dx+dy>0.05){ //Make sure sampling point not too close
				geometry_msgs::PoseStamped pose;
				pose.header = msg->header;
				pose.pose = msg->pose.pose;
				path_.poses.push_back(pose);
			}
		}
	}

	void Tracking::start() {
		active_ = true;
		tracking_thread_.swap(* new std::thread(&Tracking::tracking, this));
	}

	void Tracking::pause() {
		active_ = false;
		states_->shiftState(States::PAUSE);
	}

	void Tracking::resume() {
		if(states_->getState() == States::PAUSE){
			active_ = true;
			states_->shiftState(States::TRACKING);
		}
	}

	void Tracking::cancel() {
		cancelled_ = true;
		done_ = true;
		success_ = true;
		if(states_->getState() == States::PREPARING){
			states_->shiftState(states_->getLastState());
		}
	}

	void Tracking::tracking() {
		ros::Rate r(5);
		while(ros::ok()){
			if(cancelled_){
				ROS_INFO("Cancel tracking");
				break;
			}

			if(ready_){
				path_.header.stamp = ros::Time::now();
				path_.header.frame_id = "map";
				path_pub_.publish(path_);
			}

			r.sleep();
		}
	}
}