#include "distWrapper.h"
#include <string>
#include "ros/ros.h"
#include "strands_executive_msgs/GetExpectedTravelTime.h"

using namespace std;

double DistWrapper::dist(string p1, string p2)
{
    
    ros::NodeHandle n;
    ros::ServiceClient expectedTimeClient = n.serviceClient<strands_executive_msgs::GetExpectedTravelTime>("/mdp_plan_exec/get_expected_travel_time");
    strands_executive_msgs::GetExpectedTravelTime srv;
    srv.request.start_id= p1;
    srv.request.ltl_task = "F \"" + p2 + "\"";
    srv.request.time_of_day="all_day";
    expectedTimeClient.call(srv);
    ROS_INFO_STREAM(srv.response.travel_time);
    return srv.response.travel_time;
    

}


