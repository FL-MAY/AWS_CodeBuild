#include <ros/ros.h>
#include <scrubber_msgs/SetCleanConfig.h>
#include <scrubber_msgs/GetCleanConfig.h>
scrubber_msgs::SetCleanConfig set_;

bool get(scrubber_msgs::GetCleanConfig::Request &req,
         scrubber_msgs::GetCleanConfig::Response &res){
	res.brush = set_.request.brush;
	//ROS_INFO("Get brush %d", res.brush);
	res.squeegee = set_.request.squeegee;
	//ROS_INFO("Get squeegee %d", res.squeegee);
	res.flow = set_.request.flow;
	//ROS_INFO("Get flow %d", res.flow);
	res.vacuum = set_.request.vacuum;
	//ROS_INFO("Get vacuum %d", res.vacuum);
	return true;
}

bool set(scrubber_msgs::SetCleanConfig::Request &req,
         scrubber_msgs::SetCleanConfig::Response &res){
        ROS_INFO("#################################################");	
	if (req.brush>=-1 && req.brush<=100){
		ROS_INFO("Set brush %d", req.brush);
	        if(req.brush != -1)
			set_.request.brush = req.brush;
	}

	if (req.squeegee >= -1 || req.squeegee <= 1){
		ROS_INFO("Set squeegee %d", req.squeegee);
		if(req.squeegee != -1)
                	set_.request.squeegee = req.squeegee;
	}

	if (req.flow>=-1 && req.flow<=100) {
		ROS_INFO("Set flow %d", req.flow);
		if(req.flow != -1)
                	set_.request.flow = req.flow;
	}

	if (req.vacuum>=-1 && req.vacuum<=100) {
		ROS_INFO("Set vacuum %d", req.vacuum);
		if(req.vacuum != -1)
                	set_.request.vacuum = req.vacuum;
	}

	return true;
}

int main (int argc,  char **argv){
	ros::init(argc, argv, "MakeSense");
	ros::NodeHandle n;
	ros::ServiceServer SetService = n.advertiseService("ctlserverset", set);
	ros::ServiceServer GetService = n.advertiseService("ctlserverget", get);
	ros::spin();
	return 0;
}
