#include <cmath>
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"



int main( int argc, char** argv )
{
  ros::init(argc, argv, "Clean_Area");
  ros::NodeHandle n;
  ros::Publisher marker_sweeper = n.advertise<visualization_msgs::Marker>("visualization_sweeper", 10);
  ros::Publisher marker_vanc = n.advertise<visualization_msgs::Marker>("visualization_vanc", 10);
  ros::Rate r(30);
  int i = 0;
  int j = 0;
  while (ros::ok())
  {

    visualization_msgs::Marker sweeper_l_1;
    visualization_msgs::Marker sweeper_l_2;
    visualization_msgs::Marker vanc_l_1;
    visualization_msgs::Marker vanc_l_2;

    sweeper_l_1.type = visualization_msgs::Marker::CUBE;
    sweeper_l_2.type = visualization_msgs::Marker::CUBE;
    vanc_l_1.type = visualization_msgs::Marker::CUBE;
    vanc_l_2.type = visualization_msgs::Marker::CUBE;
    sweeper_l_1.header.frame_id = sweeper_l_2.header.frame_id = vanc_l_1.header.frame_id = vanc_l_2.header.frame_id = "base_link";
    sweeper_l_1.header.stamp = sweeper_l_2.header.stamp = vanc_l_1.header.stamp = vanc_l_2.header.stamp = ros::Time::now();
    sweeper_l_1.ns = "sweeper_l_1";
    sweeper_l_2.ns = "sweeper_l_2";
    vanc_l_1.ns = "vanc_l_1";
    vanc_l_2.ns = "vanc_l_2";
    sweeper_l_1.action = sweeper_l_2.action = vanc_l_1.action = vanc_l_2.action = visualization_msgs::Marker::ADD;
    sweeper_l_1.pose.orientation.w = sweeper_l_2.pose.orientation.w = vanc_l_1.pose.orientation.w = vanc_l_2.pose.orientation.w = 1.0;
    sweeper_l_1.lifetime = sweeper_l_2.lifetime = vanc_l_1.lifetime = vanc_l_2.lifetime = ros::Duration();
    
    sweeper_l_1.pose.position.x = 0.25;
    sweeper_l_1.pose.position.y = 0.339;
    sweeper_l_1.pose.position.z = 0;

    vanc_l_1.pose.position.x = -0.1237;
    vanc_l_1.pose.position.y = 0.4139;
    vanc_l_1.pose.position.z = 0;    

    sweeper_l_2.pose.position.x = 0.25;
    sweeper_l_2.pose.position.y = -0.339;
    sweeper_l_2.pose.position.z = 0;

    vanc_l_2.pose.position.x = -0.1237;
    vanc_l_2.pose.position.y = -0.4139;
    vanc_l_2.pose.position.z = 0; 
	
    sweeper_l_1.id = i++;
    sweeper_l_2.id = i;
    vanc_l_1.id = j++;
    vanc_l_2.id = j;
    
    i%= 1000;
    j%= 1000;
  

    
    // POINTS markers use x and y scale for width/height respectively
    sweeper_l_2.scale.x = 0.05;
    sweeper_l_2.scale.y = 0.05;
    sweeper_l_2.scale.z = 0.05;

    vanc_l_2.scale.x = 0.05;
    vanc_l_2.scale.y = 0.05;
    vanc_l_2.scale.z = 0.05;

    sweeper_l_1.scale.x = 0.05;
    sweeper_l_1.scale.y = 0.05;
    sweeper_l_1.scale.z = 0.05;

    vanc_l_1.scale.x = 0.05;
    vanc_l_1.scale.y = 0.05;
    vanc_l_1.scale.z = 0.05;

    // Points are green
    sweeper_l_1.color.g = 1.0f;
    sweeper_l_1.color.a = 1.0;
    sweeper_l_2.color.g = 1.0f;
    sweeper_l_2.color.a = 1.0;

    vanc_l_1.color.r = 1.0f;
    vanc_l_1.color.a = 1.0;
    vanc_l_2.color.r = 1.0f;
    vanc_l_2.color.a = 1.0;
    
    marker_sweeper.publish(sweeper_l_1);
    marker_vanc.publish(vanc_l_1);
    marker_sweeper.publish(sweeper_l_2);
    marker_vanc.publish(vanc_l_2);
    r.sleep();

  }
}
