#include "ros/ros.h"
#include "strands_executive_msgs/GetSchedule.h"


bool getSchedule(strands_executive_msgs::GetSchedule::Request  &req,
         			strands_executive_msgs::GetSchedule::Response &res) {
  ROS_INFO("Got a request for a schedule");
  return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "schedule_server");
	ros::NodeHandle nh;

  	ros::ServiceServer service = nh.advertiseService("get_schedule", getSchedule);
  	ROS_INFO("Ready to serve schedules");
  	ros::spin();

	return 0;
}