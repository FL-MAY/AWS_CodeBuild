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
	Automatic::Automatic(std::shared_ptr<ScrubberStates>& states, int32_t mission_id, uint8_t config, int32_t exe_times)
			: tf2_(tf_buffer_), states_(states), id_(mission_id), move_base_ol_(false), retry_(0), in_goto_(false),
			  goto_cancel_(false), exe_path_ol_(false), get_path_ol_(false), pause_(false), active_(false),
			  cross_bump_(false), goto_pause_(false), enable_avoidance_(mission_id != 0), config_id_(1),
			  movebase_(new PointCtrl("move_base_flex/move_base", true)),
			  path_ctrl_(new ExePathCtrl("move_base_flex/exe_path", true)),
			  get_path_(new GetPath("move_base_flex/get_path", true)),
			  inf_client_(new dynamic_reconfigure::Client<costmap_2d::InflationPluginConfig>(
					  "/move_base_flex/global_costmap/inflation")),
			  dwa_client_(new dynamic_reconfigure::Client<dwa_local_planner::DWAPlannerConfig>(
					  "/move_base_flex/DWAPlannerROS")),
			  teb_client_(new dynamic_reconfigure::Client<teb_local_planner::TebLocalPlannerReconfigureConfig>(
					  "/move_base_flex/TebLocalPlannerROS")) {
		if (!states) {
			ROS_ERROR("nullptr states!");
		}

		if (config > 0) {
			config_id_ = config;
		}
		//Shut done manual control
		for(int i = 0; i< 3; i++){
			if(states->gripControl(false))
				break;
			ROS_WARN("Disable grip control failed %d of 3",i+1);
		}

		//Get Tasks
		ros::NodeHandle    n;
		ros::NodeHandle    nh("~");
		ros::ServiceClient db_client = n.serviceClient<db_msgs::GetPlan>("scrubber_database/getPlan");
		db_msgs::GetPlan   srv;
		if (mission_id == -1) { //Exe all plans in this map
			db_msgs::GetListInMap get_plans;
			get_plans.request.type   = db_msgs::TypeQuery::PLAN;
			get_plans.request.map_id = states_->getMapId();
			ros::service::call("scrubber_database/getListInMap", get_plans);
			for (auto& info: get_plans.response.contents) {
				srv.request.plan_id = info.id;
				db_client.call(srv);
				tasks_.insert(tasks_.end(), srv.response.tasks.begin(), srv.response.tasks.end());
			}
		} else {
			srv.request.plan_id = mission_id;
			if (db_client.call(srv)) {
				tasks_ = srv.response.tasks;
				ROS_INFO("Got tasks");
			} else {
				ROS_ERROR("Unable to resolve this mission");
			}
		}

		//Init report
		initReport(tasks_);
		//Adding repeat tasks
		int origion_size = tasks_.size();
		ROS_INFO("exe times: %d", exe_times);

		for (int i = 1; i < exe_times; i++) {
			tasks_.insert(tasks_.end(), tasks_.begin(), tasks_.begin() + origion_size);
		}

		ROS_INFO("Task numbers : %ld", tasks_.size());

		//Subscribe
		pose_sub_        = n.subscribe("amcl_pose", 5, &Automatic::poseCallback, this);
		path_pub_        = nh.advertise<nav_msgs::Path>("tracking_path", 1, true);
		allow_backward_  = n.advertise<std_msgs::Int32>("allow_backward", 1, true);
		//Checking controllers state
		ROS_INFO("Waiting for controller ......");
		int8_t   patient = 0;

		while (!(move_base_ol_ && exe_path_ol_ && get_path_ol_) || patient++ < 2) {
			move_base_ol_ = movebase_->waitForServer(ros::Duration(1));
			exe_path_ol_  = path_ctrl_->waitForServer(ros::Duration(1));
			get_path_ol_  = get_path_->waitForServer(ros::Duration(1));
		}

		ROS_INFO("Controller is ready");

		dwa_config_ol_ = dwa_client_->getCurrentConfiguration(dwa_config_, ros::Duration(0.5));
		teb_config_ol_ = teb_client_->getCurrentConfiguration(teb_config_, ros::Duration(0.5));
		inf_config_ol_ = inf_client_->getCurrentConfiguration(inf_config_, ros::Duration(0.5));
		if (dwa_config_ol_) {
			ROS_INFO("DWA Dynamic reconfigure online");
		}

		if (teb_config_ol_) {
			ROS_INFO("TEB Dynamic reconfigure online");
		}

		if (inf_config_ol_) {
			ROS_INFO("Costmap inflation layer dynamic reconfigure online");
		}

		states_->shiftState(States::AUTOMATIC);
		// debug info
		if (!move_base_ol_) {
			ROS_ERROR("Move base controller is not online");
		}

		if (!exe_path_ol_) {
			ROS_ERROR("Exe path controller is not online");
		}

		if (!get_path_ol_) {
			ROS_ERROR("Get path is not online");
		}

		if (patient < 10) {
			ROS_INFO("All controller on line");
		}

		if (enable_avoidance_) {
			std_srvs::Empty ept;
			if(!ros::service::call("move_base_flex/clear_costmaps", ept)){
				ROS_WARN("Failed to clear mbf cost map before initializing automatic costmap");
			}

			costmap_ = std::make_shared<costmap_2d::Costmap2DROS>("automatic_costmap", tf_buffer_);
			obstacle_avoid_.swap(*new std::thread(&Automatic::obstacleAvoid, this));
		}
	}

	void Automatic::exeThread() {
		//TODO: handle level 3 Error

		states_->shiftState(States::AUTOMATIC_CLEAN);
		if (report_) {
			report_->start();
		}
		int                counter     = 0;
		int                freq        = 5;
		bool               send_path   = false;
		bool               update_task = true;
		std::vector<geometry_msgs::PoseStamped> waypoints;
		ros::NodeHandle    n;
		ros::ServiceClient task_client = n.serviceClient<db_msgs::GetTask>("scrubber_database/getTask");
		//db_msgs::GetTask   task_srv_;
		//nav_msgs::Path& cur_route_ = task_srv_.response.path;
		ros::Rate          r(freq);

		setConfig(config_id_);
		while (!tasks_.empty()) {
			done_    = false;
			success_ = false;
			allowBackward(false);
			if (states_->getState() != States::AUTOMATIC && states_->getState() != States::AUTOMATIC_CLEAN) {
				done_    = true;
				success_ = false;
				ROS_ERROR("Unable to perform task while in state : %s",
				          states_->stateName(states_->getState()).c_str());
				break;
			}

			task_srv_.request.task_id = tasks_[0].task_id;
			if (update_task && !task_client.call(task_srv_)) {
				ROS_ERROR("Unable to got task : %d", task_srv_.request.task_id);
				done_    = true;
				success_ = false;
				break;
			}

//			if (send_path) {
//				if (setConfig(task_srv_.response.config_id)) {
//					states_->setConfig(task_srv_.response.config_id);
//				} else {
//					ROS_ERROR("fail to set clean config");
//				}
//			}

			mbf_msgs::MoveBaseGoal cur_goal;
			mbf_msgs::ExePathGoal  cur_path;
			mbf_msgs::GetPathGoal  get_path_goal;
			if (cur_route_.poses.empty()) {
				ROS_ERROR("Got an empty path!");
				done_    = true;
				success_ = false;
				break;
			}

			if (cur_route_.poses[0].header.frame_id != "map") {
				ROS_ERROR("Got an pose not in map!");
				done_    = true;
				success_ = false;
				break;
			}

			if (!send_path) {
				if(waypoints.empty()) {
					if(cross_bump_){ //Just crossed bump, resume configs
						teb_client_->setConfiguration(teb_config_);
						setConfig(config_id_);
					}

					get_path_goal.target_pose = cur_route_.poses[0];
					get_path_->sendGoalAndWait(get_path_goal, ros::Duration(1),ros::Duration(1));
					if(get_path_->getResult()->outcome == mbf_msgs::GetPathResult::SUCCESS){
						ROS_WARN("GOT CANDIDATE PATH");
						scrubber_msgs::CrossBump cross_bump;
						cross_bump.request.candidate = get_path_->getResult()->path;
						if(ros::service::call("scrubber/crossBump", cross_bump)){
							ROS_WARN("ROUTE INTERSECT WITH SPEED BUMP DETOURING....");
							waypoints.insert(waypoints.begin(),
									cross_bump.response.waypoints.begin(),
									cross_bump.response.waypoints.end());
							continue;
						}
					}

					cur_goal.target_pose = cur_route_.poses[0];
					sendGoal(cur_goal); //Get to start position of cur_path
					if (cur_route_.poses.size() == 1) { //Current task is goto & already performed
						tasks_.erase(tasks_.cbegin());
						update_task = true;
					} else {
						send_path   = true;
						update_task = false;
					}
				}else{
					if(waypoints.size() == 1){
						cross_bump_ = true;
						teb_local_planner::TebLocalPlannerReconfigureConfig slow_config = teb_config_;
						slow_config.max_vel_x = 0.2;
						teb_client_->setConfiguration(slow_config);
						setConfig(1);
					}

					cur_goal.target_pose = waypoints[0];
					sendGoal(cur_goal);
					waypoints.erase(waypoints.begin());
					update_task = false;
				}
			} else {
				cur_path.path = cur_route_;
				sendGoal(cur_path);
				tasks_.erase(tasks_.cbegin());
				send_path   = false;
				update_task = true;
				new_path_   = true;
			}


			while (n.ok()) {
				if (states_->getState() == States::PAUSE || states_->getState() == States::AUTOMATIC_GOTO) {
					if (cancelled_) {
						done_    = true;
						success_ = false;
						tasks_.clear();
						if (resume_from_goto_) { //Also cancel back to path
							resume_from_goto_ = false;
							goto_cancel_      = true;
						}

						break;
					}

					if (!pause_) {//resume from pause;
						if (using_move_base_) {
							sendGoal(cur_goal);
						} else {
							double square_dist = squareDistance(resume_pose_, cur_pose_);
							ROS_INFO("Away distance is %.2f", square_dist);
							if (square_dist < 0.09) {
								sendGoal(cur_path);
								//setConfig(states_->getConfig());
							} else {
								ROS_INFO("#####x:%.2f y:%.2f frame:%s#####", resume_pose_.pose.position.x,
								         resume_pose_.pose.position.y, resume_pose_.header.frame_id.c_str());
								setWayPoint(resume_pose_);
								resume_from_goto_ = true;
								pause_            = true;
								r.sleep();
								continue;
							}
						}

						states_->shiftState(States::AUTOMATIC_CLEAN);
					}

					r.sleep();
					continue;
				}

				if (cancelled_) {
					cancelGoal();
					resetConfig();
					done_    = true;
					success_ = false;
					tasks_.clear();
					break;
				}

				if (pause_) {
					cancelGoal();
					states_->shiftState(States::PAUSE);
					//setConfig(1);//OFF
					ros::Duration(0.5).sleep();//Wait for robot stop
					ros::spinOnce();//Update current pose
					resume_pose_ = cur_pose_;
					ROS_INFO("x:%.2f y:%.2f frame:%s", resume_pose_.pose.position.x,
					         resume_pose_.pose.position.y, resume_pose_.header.frame_id.c_str());
					continue;
				}

				updateMission();

				if (counter % freq == 0) { //update every second
					updateStatistics();
				}

				// TODO: Check error level
				uint8_t error = states_->getError();
				if (error == 2) {
					//ROS_ERROR("Unable to set clean config");
				}

				if (done_) {
					//resetConfig();
					break;
				}

				counter++;
				counter %= 100;
				r.sleep();
			}

			if (cancelled_) break;
		}

		enable_avoidance_ = false;
		setConfig(1);
		states_->setConfig(1);
		if (states_->getState() == States::AUTOMATIC_GOTO) { //Cancelled while in goto
			saved_state_ = States::AUTOMATIC;
		} else {
			states_->shiftState(States::AUTOMATIC);
		}

		//Save report
		ROS_INFO("Clean area : %d cm^2", report_->getCleanArea());
		ROS_INFO("Clean Time : %d s", report_->getCleanTime());
		db_msgs::AddPath add_path;
		add_path.request.by_topic = false;
		add_path.request.map_id   = states_->getMapId();
		add_path.request.type     = db_msgs::TypePath::ROUTE;
		add_path.request.name     = "Path_Record@" + std::to_string(ros::Time::now().toSec());
		add_path.request.path     = path_;
		ros::service::call("scrubber_database/addPath", add_path);

		report_->saveReport(id_ == -1? 0 : id_, add_path.response.path_id, states_->getMapId(), config_id_);
		active_           = false;
	}

	void Automatic::moveBaseUpdate(const mbf_msgs::MoveBaseFeedbackConstPtr& cur_pose) {
		//cur_pose_ = cur_pose->current_pose;
		//ROS_INFO("Current post %.2f %.2f", cur_pose_.pose.position.x, cur_pose_.pose.position.y);
	}

	void Automatic::start() {
		if (active_) return;
		if(tasks_.empty()){
			done_ = true;
			success_ = true;
			enable_avoidance_ = false;
			ROS_ERROR("Got empty plan : %d", id_);
			return;
		}

		automatic_thread_.swap(*new std::thread(&Automatic::exeThread, this));
		active_ = true;
	}

	void Automatic::pause(bool stop_clean) {
		if (states_->getState() == States::AUTOMATIC_CLEAN) {
			ROS_INFO("Pause Clean task");
			pause_ = true;
			if(stop_clean){
				setConfig(99); //Pause clean mechanism
			}
		}

		if (states_->getState() == States::AUTOMATIC_GOTO) {
			ROS_INFO("Pause Goto task");
			goto_pause_ = true;
			if(stop_clean){
				setConfig(99); //Pause
			}
		}

		if(in_goto_ ){
			goto_pause_ = true;
		}
	}

	void Automatic::pause() {
		pause(true);
	}

	/*
	 * Function to resume paused Clean/Goto task
	 * Goto task should resume first
	 */
	void Automatic::resume() {
		if (goto_pause_) {
			setWayPoint(target_pose_.target_pose);
			goto_pause_ = false; //flag is needed for tracking old state,reset goto_pause_ after resend target pose
			return;
		}

		if (pause_) {
			setConfig(config_id_);
			pause_ = false;
			return;
		}

		ROS_INFO("Nothing paused");
	}

	Automatic::~Automatic() {
		//Resume manual control
		states_->gripControl();
		if (automatic_thread_.joinable()) {
			automatic_thread_.join();
		}

		if (goto_thread_.joinable()) {
			goto_thread_.join();
		}

		if (costmap_) {
			costmap_->stop();
		}

		if (obstacle_avoid_.joinable()) {
			obstacle_avoid_.join();
		}
	}

	void Automatic::cancelGoal() {
		if (using_move_base_) {
			ROS_INFO("Cancel current move base goal");
			movebase_->cancelGoal();
		} else {
			ROS_INFO("Cancel current exe path goal");
			path_ctrl_->cancelGoal();
		}
	}

	void Automatic::updateMission() {
		if (using_move_base_) {
			if (movebase_->getState().isDone()) {
				success_ = movebase_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
				if (success_) {
					done_  = true;
					retry_ = 0;
					return;
				}

				uint32_t outcome = movebase_->getResult().get()->outcome;

				retry_++;
				if (retry_ > 3) {
					done_ = true;
					tasks_.clear();
					return;
				}

				//TODO: try another config
				allowBackward();
				if (outcome == mbf_msgs::MoveBaseResult::OSCILLATION) {
					ROS_INFO("MOVE BASE FAILED BECAUSE OF OSCILLATION");
				} else if (outcome == mbf_msgs::MoveBaseResult::COLLISION) {
					ROS_INFO("MOVE BASE FAILED BECAUSE OF COLLISION");
				} else if (outcome == mbf_msgs::MoveBaseResult::GOAL_BLOCKED
				           || outcome == mbf_msgs::MoveBaseResult::START_BLOCKED) {
					ROS_INFO("MOVE BASE FAILED BECAUSE OF GOAL/START BLOCKED");
				} else if (outcome == mbf_msgs::MoveBaseResult::PLAN_FAILURE) {
					ROS_INFO("MOVE BASE FAILED BECAUSE OF PLAN FAILURE");
				} else if (outcome == 105) {
					ROS_INFO("MOVE BASE FAILED BECAUSE OF LOCAL PLANNER OSCILLATION");
				} else {
					ROS_INFO("MOVE BASE FAILED BECAUSE OF %d", outcome);
					done_ = true;
					tasks_.clear();
					return;
				}
				//Retry
				states_->shiftState(States::PAUSE);
			}
		} else {
			if (path_ctrl_->getState().isDone()) {
				success_ = path_ctrl_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
				if (success_) {
					ROS_INFO("Follow path success");
					done_  = true;
					retry_ = 0;
					return;
				}

				uint32_t outcome = path_ctrl_->getResult().get()->outcome;
				retry_++;
				if (retry_ > 5) {
					done_ = true;
					tasks_.clear();
					ROS_INFO("Exceed max retry time...");
					return;
				}

				allowBackward();
				if (outcome == mbf_msgs::ExePathResult::OSCILLATION) {
					ROS_INFO("EXE PATH FAILED BECAUSE OF OSCILLATION, RETRY...");
				} else if (outcome == mbf_msgs::ExePathResult::COLLISION) {
					ROS_INFO("EXE PATH FAILED BECAUSE OF COLLISION, RETRY...");
				} else if (outcome == mbf_msgs::ExePathResult::MISSED_PATH
				           || outcome == mbf_msgs::ExePathResult::MISSED_GOAL) {
					ROS_INFO("EXE PATH FAILED BECAUSE OF GOAL/START MISSED, RETRY...");
				} else if (outcome == mbf_msgs::ExePathResult::BLOCKED_PATH) {
					ROS_INFO("EXE PATH FAILED BECAUSE OF BLOCKED PATH, RETRY...");
				} else if (outcome == mbf_msgs::ExePathResult::ROBOT_STUCK) {
					ROS_INFO("EXE PATH FAILED BECAUSE OF ROBOT STUCK, RETRY...");

				} else if (outcome == mbf_msgs::ExePathResult::PAT_EXCEEDED) {
					ROS_INFO("MOVE BASE FAILED BECAUSE OF PAT EXCEEDED. RETRY...");

				} else {
					ROS_INFO("EXE PATH FAILED BECAUSE OF %d", outcome);
					done_ = true;
					tasks_.clear();
					return;
				}
				//Retry
				resume_pose_     = cur_pose_; //update pause config
				states_->shiftState(States::PAUSE);
			}
		}
	}

	bool Automatic::sendGoal(mbf_msgs::MoveBaseGoal& target) {
		if (target.target_pose.header.frame_id != "map") {
			ROS_ERROR("Unknown frame id %s, x: %.2f  y: %.2f", target.target_pose.header.frame_id.c_str(),
			          target.target_pose.pose.position.x, target.target_pose.pose.position.y);
			return false;
		}

		if (!move_base_ol_) {
			ROS_ERROR("Move base controller not online!");
			return false;
		}

		using_move_base_ = true;
		movebase_->sendGoal(target,
		                    PointCtrl::SimpleDoneCallback(),
		                    PointCtrl::SimpleActiveCallback(),
		                    boost::bind(&Automatic::moveBaseUpdate, this, _1));

		return true;
	}

	bool Automatic::sendGoal(mbf_msgs::ExePathGoal& pathGoal) {

		if (pathGoal.path.header.frame_id != "map" || pathGoal.path.poses[0].header.frame_id != "map") {
			ROS_ERROR("Send path unknown frame id %s", pathGoal.path.header.frame_id.c_str());
			return false;
		}
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
		config.request.brush    = 0;
		config.request.squeegee = 0;
		config.request.vacuum   = 0;
		config.request.flow     = 0;
		if (!ros::service::call("ctlserverset", config)) {
			ROS_ERROR("Fail to reset clean config");
			states_->setError(2);
			return false;
		}
		return true;
	}

	bool Automatic::setConfig(uint32_t config_id) {
		scrubber_msgs::SetPathConfig config;
		config.request.config_id = config_id;
		if (ros::service::call("scrubber/setPathConfig", config)) {
			states_->setConfig(config_id);
			return true;
		}

		states_->setError(2);
		return false;
	}

	void Automatic::setWayPoint(geometry_msgs::PoseStamped& target_pose) {
		if (active_ && states_->getState() != States::PAUSE && states_->getState() != States::AUTOMATIC_GOTO) {
			ROS_ERROR("Can not perform goto now, please pause current mission first");
			return;
		}

		target_pose_.target_pose = target_pose;
		new_goal_ = true;
		if (!in_goto_) {
			if (!goto_pause_) {
				saved_state_      = states_->getState();
				saved_controller_ = using_move_base_;
				ROS_INFO("State before goto %s", states_->stateName(saved_state_).c_str());
			}
			goto_thread_.swap(*new std::thread(&Automatic::gotoThread, this));
		}
	}

	void Automatic::obstacleAvoid() {
		int       step       = 30;
		int       start_idx  = 0;
		int       cur_idx    = -1;
		int       target_idx = -1;
		bool      detour     = false;
		ros::Rate r(2);
		while (ros::ok()) {
			if (!enable_avoidance_) {
				break;
			}

			ros::spinOnce();
			r.sleep();


			if (costmap_) {
				costmap_->updateMap();
			}

			if (states_->getState() != States::AUTOMATIC_CLEAN && states_->getState() != States::AUTOMATIC_GOTO &&
			    states_->getState() != States::PAUSE) {
				continue;
			}

			if (new_path_) {
				ROS_INFO("New path");
				start_idx = 0;
				cur_idx   = -1;
				new_path_ = false;
			}

			if (detour) {
				start_idx = target_idx;
				if(goto_pause_){
					ROS_INFO_ONCE("Pausing detour");
					continue;
				}

				if (!isFree(cur_route_.poses[target_idx])) { //Target is in Obs try find another way point
					ROS_WARN("Target pose in obs");
					collisionAhead(cur_route_, start_idx, target_idx); //update target index;
					if (target_idx != -1) {
						setWayPoint(cur_route_.poses[target_idx]);
						continue;
					}

					ROS_WARN("Can't find free space as alternative target pose");
				}

				if (movebase_->getState().isDone() || states_->getState() == States::PAUSE) {
					ROS_WARN("Detour finished!");
					inf_config_.inflation_radius -= 0.3;
					inf_client_->setConfiguration(inf_config_);
					allowBackward(false);
					detour       = false;
					resume_pose_ = cur_pose_;
					resume();
					start_idx = movebase_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED ?
							    target_idx : start_idx;
				}

			} else if (states_->getState() == States::AUTOMATIC_CLEAN) { //!!using_move_base_
				cur_idx = indexOfPose(cur_route_, cur_pose_, start_idx);
				if (cur_idx == -1) {
					ROS_WARN_ONCE("Could not found robot pose in current route");
					continue;
				}

				start_idx = cur_idx;
				if (collisionAhead(cur_route_, start_idx, target_idx) && target_idx != -1) {
					if(states_ -> getState() == States::PAUSE) {
						ROS_WARN("Pausing cancel detour...");
						continue;
					}

					pause(false);
					inf_config_.inflation_radius += 0.3;
					inf_client_->setConfiguration(inf_config_);
					allowBackward();
					ros::Duration(0.5).sleep();//Wait for robot pause;
					setWayPoint(cur_route_.poses[target_idx]);
					detour = true;
				}
			}
		}
	}

	void Automatic::gotoThread() {
		states_->shiftState(States::AUTOMATIC_GOTO);
		in_goto_     = true;
		goto_pause_  = false;
		goto_cancel_ = false;
		auto sent = sendGoal(target_pose_);


		new_goal_ = false;
		retry_    = 0;
		ros::Rate r(5);
		while (ros::ok()) {
			if (!sent) {
				ROS_ERROR("Error while sending goal");
				break;
			}

			if (new_goal_) {
				ROS_INFO("Send new target goal");
				sent      = sendGoal(target_pose_);
				new_goal_ = false;
				r.sleep();
				continue;
			}

			if (movebase_->getState().isDone()) {
				if (movebase_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
					in_goto_ = false;
					if (resume_from_goto_) {
						resume_from_goto_ = false;
						pause_            = false;
						using_move_base_  = false;//resume_from_goto is set by path task
					}
					break;
				}

				uint32_t outcome = movebase_->getResult().get()->outcome;

				retry_++;
				if (retry_ > 0) {
					in_goto_ = false;
					states_->setError(3);//TODO: Goto failed error
					if (resume_from_goto_) {
						resume_from_goto_ = false;
						using_move_base_  = false;//resume_from_goto is set by path task
					}
					break;
				}
				//TODO: retry goto
			}

			if (goto_pause_) {
				movebase_->cancelGoal();
				in_goto_ = false;
				states_->shiftState(States::PAUSE);
				return;
			}

			if (goto_cancel_) {
				movebase_->cancelGoal();
				break;
			}

			r.sleep();
		}

		states_->shiftState(saved_state_);
		using_move_base_ = saved_controller_;
		in_goto_         = false;
	}

	void Automatic::resumeGoto() {
		if (!goto_pause_) return;
		goto_pause_ = false;
		setWayPoint(target_pose_.target_pose);
	}

	void Automatic::cancelGoto() {
		goto_cancel_ = true;
		goto_pause_  = false;
	}

	void Automatic::poseCallback(geometry_msgs::PoseWithCovarianceStamped::ConstPtr const& msg) {
		ROS_INFO_ONCE("Got amcl pose");
		cur_pose_.pose   = msg->pose.pose;
		cur_pose_.header = msg->header;
		if (report_ && states_->getState() == States::AUTOMATIC_CLEAN) {
			report_->paint(cur_pose_.pose.position.x, cur_pose_.pose.position.y);
		}

		if (states_->getState() == States::AUTOMATIC_CLEAN || states_->getState() == States::AUTOMATIC_GOTO) {
			path_.poses.push_back(cur_pose_);
			path_pub_.publish(path_);
		}
	}

	double Automatic::squareDistance(geometry_msgs::PoseStamped& p1, geometry_msgs::PoseStamped& p2) {
		double dx = fabs(p1.pose.position.x - p2.pose.position.x);
		double dy = fabs(p1.pose.position.y - p2.pose.position.y);
		return dx * dx + dy * dy;
	}

	int Automatic::indexOfPose(nav_msgs::Path& path, geometry_msgs::PoseStamped& pose, int start_index) {
		int    path_size = path.poses.size();
		double threshold = 0.15 * 0.15; //Distance threshold to belive we are at that location of path
		if (start_index >= path_size) {
			return -1;
		}

		for (int idx = start_index; idx < path_size; idx++) {
			if (squareDistance(path.poses[idx], pose) < threshold) {
				ROS_DEBUG("Found at %d of path %d", idx, path_size);
				return idx;
			}
		}

		return -1;
	}

	void Automatic::initReport(std::vector<db_msgs::Task>& tasks) {
		if (tasks.empty()) return;
		double min_x, max_x, min_y, max_y;
		min_x = min_y = 1000;
		max_x = max_y = -1000;
		db_msgs::GetTask get_task;
		for (auto& task : tasks) {
			get_task.request.task_id = task.task_id;
			ros::service::call("scrubber_database/getTask", get_task);
			for (auto& pose : get_task.response.path.poses) {
				if (pose.pose.position.x < min_x) {
					min_x = pose.pose.position.x;
				}

				if (pose.pose.position.x > max_x) {
					max_x = pose.pose.position.x;
				}

				if (pose.pose.position.y < min_y) {
					min_y = pose.pose.position.y;
				}

				if (pose.pose.position.y > max_y) {
					max_y = pose.pose.position.y;
				}
			}
		}

		report_ = std::make_unique<Report>(min_x, min_y, max_x, max_y, 0.05);
	}

	void Automatic::updateStatistics() {
		if (!report_) return;
		scrubber_msgs::UpdateStatistics ups;
		ups.request.clean_time_increment = report_->getTimeIncre();
		ups.request.clean_area_increment = report_->getAreaIncre();
		ros::service::call("scrubber/updateStatistics", ups);
	}

	bool Automatic::collisionAhead(nav_msgs::Path& path, int start_index, int& next_free_idx) {
		int step_size   = 1;
		int window_size = start_index + 65;
		int path_size   = path.poses.size();
		if (start_index >= path_size) {
			ROS_WARN("start index over path size in when checking whether safe ahead");
			return false;
		}

		for (int idx = start_index; idx < window_size; idx += step_size) {
			if (idx >= path_size) { //safe ahead
				break;
			}

			if (!isFree(path.poses[idx])) {
				ROS_WARN("Obs detected!");
				for (int check = idx + step_size; check < path_size; check += step_size) {
					if (check >= path_size) {
						break;
					}

					if (isFree(path.poses[check])) {
						next_free_idx = std::min(check + 20, path_size - 1);
						ROS_WARN("Found next free idx %d", next_free_idx);
						return true;
					}
				}

				next_free_idx = -1; //Did not found any free space ahead
				return true;
			}

		}

		return false;
	}

	bool Automatic::isFree(geometry_msgs::PoseStamped& pose) {
		uint32_t mx, my;
		if (!costmap_->getCostmap()->worldToMap(pose.pose.position.x,
		                                        pose.pose.position.y,
		                                        mx, my)) {
			ROS_WARN("Can't find path way point in costmap");
			return false;
		}

		return costmap_->getCostmap()->getCost(mx, my) == costmap_2d::FREE_SPACE;
	}

	void Automatic::allowBackward(bool allow){
		std_msgs::Int32 msg;
		msg.data = allow ? 1 : 0;
		allow_backward_.publish(msg);
	}
}//namespace
