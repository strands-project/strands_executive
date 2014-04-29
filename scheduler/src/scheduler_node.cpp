#include "ros/ros.h"
#include "strands_executive_msgs/GetSchedule.h"
#include "task.h"
#include "scheduler.h"
#include <vector>
#include <algorithm>

using namespace std;


// /*constructor without parameter now, automatically set now to false*/
// Task::Task(unsigned int ID, double s, double e, double d, string start_pos, string end_pos)
// {
//   id = ID;
//   start = s;
//   end = e;
//   duration = d;
//   s_pos = start_pos;
//   e_pos = end_pos;
//   no = false;
//   cond = false;
// }


Task * createSchedulerTask(const strands_executive_msgs::Task & _task) {

	Task* t = new Task(_task.task_id,
						_task.start_after.toSec(),
						_task.end_before.toSec(),
						_task.max_duration.toSec(),
						_task.start_node_id,
						_task.end_node_id);

	return t;
}

// bool compareTasks (const Task * i, const Task * j) { 
bool compareTasks ( Task * i,  Task * j) { 
	return (i->getExecTime()<j->getExecTime()); 
}


bool getSchedule(strands_executive_msgs::GetSchedule::Request  &req,
         			strands_executive_msgs::GetSchedule::Response &res) {
  
  ROS_INFO_STREAM("Got a request for a schedule " << req.tasks.size() << " tasks ");

  std::vector<Task*> tasks;

  // for(strands_executive_msgs::Task task : req.tasks) {
  for(auto & task : req.tasks) {
    ROS_INFO_STREAM(task.task_id);    
  	tasks.push_back(createSchedulerTask(task));
  }

  Scheduler scheduler(&tasks);
  if(scheduler.solve()) {
  	std::sort(tasks.begin(), tasks.end(), compareTasks);

  	for(auto & tp : tasks) {
  		res.task_order.push_back(tp->getID());
  		res.execution_times.push_back(ros::Time(tp->getExecTime()));
  		delete tp;
  	} 

  }
  else {

	  // manage memory explicitly until Lenka changes to smart pointers
	  for(auto & tp : tasks) {
	  	delete tp;
	  } 

	}
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