//
// Created by rock-trl on 9/26/19.
//

#include <boost/shared_ptr.hpp>
#include <cmath>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Twist.h>
#include <navigator/CmdVelSmootherConfig.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#define ZERO 0.0001

template<class T> inline T sgn(T val) { return val > 0 ? 1.0 : -1.0; }

namespace dyn = dynamic_reconfigure;

class VelocitySmootherNode
{
	ros::NodeHandle nh_, pnh_;
	ros::Publisher  pub_;
	ros::Subscriber sub_;
	ros::Subscriber ratio_sub_;
	ros::Subscriber backward_sub_;
	ros::Timer timer_;

	boost::mutex cfg_mutex_;
	typedef cmd_vel_smoother::CmdVelSmootherConfig Config;
	boost::shared_ptr<dyn::Server<Config> > dyn_srv_;

	bool stop_flag_;
	int interpolate_max_frame_, count_;
	int smoother_step_, step_;
	double actual_rate_, desired_rate_;  //desired_rate_ suppose to be larger than actual_rate_!!!
	double x_acc_lim_, y_acc_lim_, yaw_acc_lim_;
	double x_step_, y_step_, yaw_step_;
	ros::Time latest_time_, timer_time_;
	geometry_msgs::Twist::ConstPtr latest_vel_;
	geometry_msgs::Twist::Ptr current_vel_, pub_vel_;
	float speed_ratio;
	int   allow_backward; // 0 --> not allow backward motion

public:
	ros::Duration timerDuration() {
		return ros::Duration(1./(desired_rate_ * 2.2));
	}

	void dynConfigCallback(Config &cfg, uint32_t level) {
		boost::mutex::scoped_lock lock(cfg_mutex_);
		x_acc_lim_ = cfg.x_acc_lim;
		y_acc_lim_ = cfg.y_acc_lim;
		yaw_acc_lim_ = cfg.yaw_acc_lim;
		interpolate_max_frame_ = cfg.interpolate_max_frame;

		if(desired_rate_ != cfg.desired_rate) {
			desired_rate_ = cfg.desired_rate;
			if (timer_.isValid()) {
				ros::Duration d = timerDuration();
				timer_.setPeriod(d);
				ROS_INFO_STREAM("timer loop rate is changed to " << 1.0 / d.toSec() << "[Hz]");
			}
		}
	}

	void velCallback(const geometry_msgs::Twist::ConstPtr &msg){
//		actual_rate_ = 1.0 / (ros::Time::now() - latest_time_).toSec();
		latest_time_ = ros::Time::now();
		latest_vel_ = msg;
		count_ = 0;
		step_ = 0;
//		smoother_step_ = int (desired_rate_ / actual_rate_);
		x_step_ = std::abs((latest_vel_->linear.x * speed_ratio - current_vel_->linear.x) / smoother_step_);
//		y_step_ = std::abs((latest_vel_->linear.y * speed_ratio - current_vel_->linear.y) / smoother_step_);
		yaw_step_ = std::abs((latest_vel_->angular.z * speed_ratio - current_vel_->angular.z) / smoother_step_);
		stop_flag_ = std::abs(latest_vel_->linear.x * speed_ratio) < ZERO &&
				     /*std::abs(latest_vel_->linear.y * speed_ratio) < ZERO &&*/
		             std::abs(latest_vel_->angular.z * speed_ratio) < ZERO;
	}

	void timerCallback(const ros::TimerEvent& event){
		if (count_ > interpolate_max_frame_) return;
		if (latest_vel_ == NULL) return;

		double past_sec = (event.current_real - latest_time_).toSec();

		if (stop_flag_ &&
			std::abs(current_vel_->linear.x) < ZERO &&
		    /*std::abs(current_vel_->linear.y) < ZERO &&*/
		    std::abs(current_vel_->angular.z) < ZERO) return;

		if (stop_flag_ &&
		    std::abs(current_vel_->linear.x) < x_acc_lim_ / desired_rate_ &&
		    /*std::abs(current_vel_->linear.y) < y_acc_lim_ / desired_rate_ &&*/
		    std::abs(current_vel_->angular.z) < yaw_acc_lim_ / desired_rate_) {
			current_vel_->linear.x = current_vel_->linear.y = current_vel_->angular.z = 0.0;
			ROS_INFO_STREAM("stop!!  " << current_vel_->linear.x << ",  " << current_vel_->angular.z << "< " << x_acc_lim_ / desired_rate_ << "  " << yaw_acc_lim_ / desired_rate_);
			pub_.publish(*current_vel_);
			count_++;
			return;
		}

		if (past_sec > step_  / desired_rate_ && step_ <= smoother_step_) {
			publishCurrentVel();
			timer_time_ = event.current_real;
			count_++;
			step_++;
			return;
		}

		if (stop_flag_) {
			current_vel_->linear.x = current_vel_->linear.y = current_vel_->angular.z = 0.0;
			pub_.publish(*current_vel_);
			count_++;
			return;
		}
	}

	void publishCurrentVel() {
		double x_gain = (x_step_ > x_acc_lim_ / desired_rate_) ? (x_acc_lim_ / desired_rate_) : x_step_;
//		double y_gain = (y_step_ > y_acc_lim_ / desired_rate_) ? (y_acc_lim_ / desired_rate_) : y_step_;
		double yaw_gain = (yaw_step_ > yaw_acc_lim_ / desired_rate_) ? (yaw_acc_lim_ / desired_rate_) : yaw_step_;
		current_vel_->linear.x += x_gain  * sgn(latest_vel_->linear.x * speed_ratio - current_vel_->linear.x);
//		current_vel_->linear.y += y_gain  * sgn(latest_vel_->linear.y * speed_ratio - current_vel_->linear.y);
		current_vel_->linear.y = 0.0;
		current_vel_->angular.z += yaw_gain  * sgn(latest_vel_->angular.z * speed_ratio - current_vel_->angular.z);
		//BUG#173 forbid backward motion
		if (current_vel_->linear.x < 0.0 && allow_backward == 0) {
//			ROS_WARN("Detect negative x velocity! Backward motion is forbidden!");
			current_vel_->linear.x = 0.0;
			if (std::abs(current_vel_->angular.z) < 0.2) {
				current_vel_->angular.z = sgn(current_vel_->angular.z) * 0.2;
			}
		}
		//BUG#120 wrong cmd_vel
		if (std::abs(current_vel_->linear.x) > 1.5 || std::abs(current_vel_->angular.z) > 1.5) {
			ROS_ERROR("wrong cmd_vel");
			ROS_ERROR("cmd_vel %.2f, %.2f; mbf_vel %.2f, %.2f; gain %.2f, %.2f; step %.2f, %.2f",
			          current_vel_->linear.x, current_vel_->angular.z,
			          latest_vel_->linear.x, latest_vel_->angular.z,
			          x_gain, yaw_gain, x_step_, yaw_step_);
			return;
		}

		pub_.publish(*current_vel_);
	}

	void speed_ratio_Callback(const std_msgs::Float32& ratio) {
		if (ratio.data < 0.0 || ratio.data > 1.0) {
			ROS_INFO("Wrong value!");
		} else {
			speed_ratio = ratio.data;
		}
	}

	void allow_backward_Callback(const std_msgs::Int32& allow) {
		if (allow.data == 0) {
			ROS_INFO("backward motion is forbidden.");
		} else if (allow.data == 1) {
			ROS_INFO("backward motion is allowed.");
		} else {
			return;
		}

		allow_backward = allow.data;
	}

	VelocitySmootherNode(): nh_(), pnh_("~") {
		latest_time_ = ros::Time::now();
		desired_rate_ = 50.0;
		ros::param::get("move_base_flex/controller_frequency", actual_rate_);
		smoother_step_ = int (desired_rate_ / actual_rate_);
		ROS_INFO("smooth step is %d", smoother_step_);

		speed_ratio = 1.0;
		ratio_sub_ = nh_.subscribe("speed_ratio", 1, &VelocitySmootherNode::speed_ratio_Callback, this);

		allow_backward = 0;
		backward_sub_ = nh_.subscribe("allow_backward", 1, &VelocitySmootherNode::allow_backward_Callback, this);

		dyn_srv_ = boost::make_shared<dyn::Server<Config>>(pnh_);
		dyn::Server<Config>::CallbackType f = boost::bind(&VelocitySmootherNode::dynConfigCallback, this, _1, _2);
		dyn_srv_->setCallback(f);

		pub_vel_ = boost::shared_ptr<geometry_msgs::Twist>(new geometry_msgs::Twist());
		current_vel_ = boost::shared_ptr<geometry_msgs::Twist>(new geometry_msgs::Twist());
		current_vel_->linear.x = current_vel_->linear.y = current_vel_->angular.z = 0.0;
		pub_ = nh_.advertise<geometry_msgs::Twist>("output", 1);
		sub_ = nh_.subscribe("input", 1, &VelocitySmootherNode::velCallback, this);
		timer_ = nh_.createTimer(timerDuration(), &VelocitySmootherNode::timerCallback, this);
	};
	~VelocitySmootherNode() {};

};

int main(int argc, char** argv) {
	ros::init(argc, argv, "cmd_vel_smoother");
	VelocitySmootherNode n;
	ros::spin();

	return 0;
}
