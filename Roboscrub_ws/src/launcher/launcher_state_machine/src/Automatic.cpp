//*********************************************************
//
// Copyright (c) 2019 Roborock. All rights reserved.
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************


#include "launcher_state_machine/Automatic.h"

namespace rock::scrubber::launcher {
	Automatic::Automatic(std::shared_ptr<ScrubberStates>& states, uint32_t mission_id)
			: states_(states), id_(mission_id), move_base_ol_(false),
			  exe_path_ol_(false), get_path_ol_(false), pause_(false),active_(false),
			  movebase_(new PointCtrl("move_base_flex/move_base", true)),
			  path_ctrl_(new ExePathCtrl("move_base_flex/exe_path", true)),
			  get_path_(new GetPath("move_base_flex/get_path", true)) {
		if (!states) {
			ROS_ERROR("nullptr states!");
		}

		//Get Tasks
		ros::NodeHandle    n;
		ros::ServiceClient db_client = n.serviceClient<db_msgs::GetPlan>("scrubber_database/getPlan");
		db_msgs::GetPlan   srv;
		srv.request.by_plan_name   = false;
		srv.request.plan_id        = mission_id;
		if (db_client.call(srv)) {
			tasks_ = srv.response.tasks;
			ROS_INFO("Got tasks");
		} else {
			ROS_ERROR("Unable to resolve this mission");
		}

		//Checking controllers state
		ROS_INFO("Waiting for controller ......");
		int8_t             patient = 0;
		while (!(move_base_ol_ && exe_path_ol_ && get_path_ol_) || patient++ < 10) {
			move_base_ol_ = movebase_->waitForServer(ros::Duration(2));
			exe_path_ol_  = path_ctrl_->waitForServer(ros::Duration(2));
			get_path_ol_  = get_path_->waitForServer(ros::Duration(2));
		}

		ROS_INFO("Controller is ready");
		states_->shiftState(States::AUTOMATIC);
		// debug info
		if (!move_base_ol_) {
			ROS_INFO("Move base controller is not online");
		}

		if (!exe_path_ol_) {
			ROS_INFO("Exe path controller is not online");
		}

		if (!get_path_ol_) {
			ROS_INFO("Get path is not online");
		}

		if (patient < 10) {
			ROS_INFO("All controller is on line");
		}
	}

	void Automatic::exeThread() {
		//TODO: handle level 3 Error
		if (tasks_.empty()) {
			done_    = true;
			success_ = true;
			ROS_ERROR("Got empty plan : %d", id_);
			return;
		}

		bool               send_path   = false;
		bool               update_task = true;
		ros::NodeHandle    n;
		ros::ServiceClient task_client = n.serviceClient<db_msgs::GetTask>("scrubber_database/getTask");
		db_msgs::GetTask   task_srv;
		nav_msgs::Path& cur_task = task_srv.response.path;

		while (!tasks_.empty()) {
			if (states_->getState() != States::AUTOMATIC) {
				done_    = true;
				success_ = false;
				ROS_ERROR("Un able to perform task while in state : %d", states_->getState());
				return;
			}

			task_srv.request.task_id = tasks_[0];
			if (update_task && !task_client.call(task_srv)) {
				ROS_ERROR("Unable to got task : %d", task_srv.request.task_id);
				done_    = true;
				success_ = false;
				return;
			}

			if (send_path) {
				if(setConfig(task_srv.response.config_id)){
					states_->setConfig(task_srv.response.config_id);
				}
				else{
					ROS_ERROR("fail to set clean config");
				}
			}

			mbf_msgs::MoveBaseGoal cur_goal;
			mbf_msgs::ExePathGoal  cur_path;
			if (cur_task.poses.empty()) {
				ROS_ERROR("Got an empty path!");
				done_    = true;
				success_ = false;
				return;
			}

			if (!send_path) {
				cur_goal.target_pose = cur_task.poses[0];
				sendGoal(cur_goal); //Get to start position of cur_path
				if (cur_task.poses.size() == 1) { //Current task is goto & already performed
					tasks_.erase(tasks_.cbegin());
					update_task = true;
				} else {
					send_path   = true;
					update_task = false;
				}

			} else {
				cur_path.path = cur_task;
				sendGoal(cur_path);
				tasks_.erase(tasks_.cbegin());
				send_path   = false;
				update_task = true;
			}

			ros::Rate r(20);
			while (n.ok()) {
				if (states_->getState() == States::PAUSE) {
					if (cancelled_) {
						done_    = true;
						success_ = false;
						states_->shiftState(States::AUTOMATIC);
						break;
					} else if (!pause_) {//resume from pause;
						if (using_move_base_) {
							sendGoal(cur_goal);
						} else {
							sendGoal(cur_path);
							setConfig(states_->getConfig());
						}

						states_->shiftState(States::AUTOMATIC);
					}
					continue;
				}

				if (pause_) {
					cancelGoal();
					states_->shiftState(States::PAUSE);
					resetConfig();
					continue;
				}

				if (cancelled_) {
					cancelGoal();
					resetConfig();
					done_    = true;
					success_ = false;
					break;
				}

				updateMission();

				// TODO: Check error level
				uint8_t error = states_->getError();
				if(error == 2){
					//ROS_ERROR("Unable to set clean config");
				}

				if (done_){
					resetConfig();
					states_->setConfig(0);
					break;
				}
				r.sleep();
			}

			if (cancelled_) break;
		}
	}

	void Automatic::moveBaseUpdate(const mbf_msgs::MoveBaseFeedbackConstPtr& cur_pose) {
		cur_pose_ = cur_pose->current_pose;
		//ROS_INFO("Current post %.2f %.2f", cur_pose_.pose.position.x, cur_pose_.pose.position.y);
	}

	void Automatic::start() {
		if (active_) return;
		automatic_thread_.swap(*new std::thread(&Automatic::exeThread, this));
		active_ = true;
	}


	void Automatic::pause() {
		if (states_->getState() == States::AUTOMATIC) {
			pause_ = true;
			return;
		}

		ROS_INFO("Currently can't pause");
	}

	void Automatic::resume() {
		if (pause_) {
			pause_ = false;
			return;
		}

		ROS_INFO("Currently nothing pause");
	}

	Automatic::~Automatic() {
		if (automatic_thread_.joinable()) {
			automatic_thread_.join();
		}
	}

	void Automatic::cancelGoal() {
		ROS_INFO("Automatic cancel goal");
		if (using_move_base_) {
			movebase_->cancelGoal();
		} else {
			path_ctrl_->cancelGoal();
		}
	}

	void Automatic::updateMission() {
		if (using_move_base_) {
			done_ = movebase_->getState().isDone();
			if (done_) {
				success_ = movebase_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
			}
		} else {
			done_ = path_ctrl_->getState().isDone();
			if (done_) {
				success_ = path_ctrl_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
			}
		}
	}

	bool Automatic::sendGoal(mbf_msgs::MoveBaseGoal& target) {
		if (!move_base_ol_) {
			ROS_ERROR("Move base controller not online!");
			return false;
		}
		using_move_base_ = true;
		movebase_->sendGoal(target,
		                    PointCtrl::SimpleDoneCallback(),
		                    PointCtrl::SimpleActiveCallback(),
		                    boost::bind(&Automatic::moveBaseUpdate, this, _1));
	}

	bool Automatic::sendGoal(mbf_msgs::ExePathGoal& pathGoal) {
		if (!exe_path_ol_) {
			ROS_ERROR("Exe path controller not online");
			return false;
		}
		using_move_base_ = false;
		path_ctrl_->sendGoal(pathGoal);
		return true;
	}

	bool Automatic::resetConfig() {
		scrubber_msgs::SetCleanConfig config;
		config.request.brush = 0;
		config.request.squeegee = 0;
		config.request.vacuum = 0;
		config.request.flow = 0;
		if(!ros::service::call("ctlserverset",config)){
			ROS_ERROR("Fail to reset clean config");
			states_->setError(2);
			return false;
		}
		return true;
	}

	bool Automatic::setConfig(uint32_t config_id) {
		scrubber_msgs::SetPathConfig config;
		config.request.config_id = config_id;
		if(ros::service::call("scrubber/setPathConfig",config)){
			states_->setConfig(config_id);
			return true;
		}

		states_->setError(2);
		return false;
	}
}//namespace
