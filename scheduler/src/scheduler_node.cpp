#include "ros/ros.h"
#include "strands_executive_msgs/GetSchedule.h"
#include "task.h"
#include "scheduler.h"
#include <vector>
#include <algorithm>
#include "mongodb_store_msgs/StringPairList.h"
#include "mongodb_store/message_store.h"

using namespace std;
using namespace mongodb_store;
using namespace mongodb_store_msgs;

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

bool save_problems = true;
int scheduler_version = -1;
string output_file = "";
int timeout = 0;

Task * createSchedulerTask(const strands_executive_msgs::Task & _task, const ros::Time & _earliestStart, const bool & _demand=false) {


  auto startAfter = _earliestStart > _task.start_after ? _earliestStart : _task.start_after;

  //ROS_INFO_STREAM("" << _earliestStart);
  //ROS_INFO_STREAM("" <<  startAfter);
  //ROS_INFO_STREAM("" << _task.start_after);
  //ROS_INFO_STREAM("" << _task.end_before);
  
  //ensure, that task have valid time window
  if(startAfter.toSec() <= (_task.end_before.toSec() - _task.max_duration.toSec()))
  {

    Task* t = new Task(_task.task_id,
						startAfter.toSec(),
						_task.end_before.toSec(),
						_task.max_duration.toSec(),
						_task.start_node_id,
						_task.end_node_id, 
            _demand);
    return t;
  }
  else //task is not valid
    return NULL;
  
}

// bool compareTasks (const Task * i, const Task * j) { 
bool compareTasks ( Task * i,  Task * j) { 
	return (i->getExecTime()<j->getExecTime()); 
}

double ** createDurationArray(const strands_executive_msgs::DurationMatrix & dm, double & max) {

  double ** array = new double *[dm.durations.size()];
  
  for(int i = 0; i < dm.durations.size(); ++i) {
    const strands_executive_msgs::DurationList & dl(dm.durations[i]);
    array[i] = new double [dl.durations.size()];
    for (int j = 0; j < dl.durations.size(); ++j)
    {
      array[i][j] = dl.durations[j].toSec();
    }
  }

  return array;
}

void clean_matrix(double ** duration_array, int size)
{
  for (int i = 0; i < size; ++i)
  {
    delete duration_array[i];
  }
  delete duration_array;
}

bool getSchedule(strands_executive_msgs::GetSchedule::Request  &req,
         			strands_executive_msgs::GetSchedule::Response &res) {
  
  ROS_INFO_STREAM("SCHEDULER: Got a request for a schedule " << req.tasks.size() << " tasks ");

  static ros::NodeHandle nh;


  std::vector<Task*> tasks;



  // for(strands_executive_msgs::Task task : req.tasks) {
  for(auto & task : req.tasks) {
    //ROS_INFO_STREAM(task.task_id << " start " << task.start_after << ", end " << task.end_before
    //     << ", duration " << task.max_duration);    

    bool first = task.task_id == req.first_task ? true : false;

    Task * sch_task = createSchedulerTask(task, req.earliest_start, first);
    
    if (sch_task != NULL)   
      tasks.push_back(sch_task);
    else
    {
      ROS_INFO("SCHEDULER: some task have invalid startime and deadline, probably it is invalid because of currently executed tasks");
      for(auto & tp : tasks) {
        delete tp;
      } 
      return true;
    }

        
    if(first) {
      ROS_INFO_STREAM("SCHEDULER:"<< task.task_id << " is first");
    }
  }


  if(save_problems) { 

    static MessageStoreProxy messageStore(nh, "scheduling_problems");
    
    vector< pair<string, string> > stored;

    for(auto & task : req.tasks) {
      static string taskType(get_ros_type(task));
      stored.push_back( make_pair(taskType, messageStore.insert(task)) );
    }

    StringPairList spl;
    for(auto & pair : stored) {
      spl.pairs.push_back(mongodb_store::makePair(pair.first, pair.second));
    }
    messageStore.insert(spl);
  }


  double max_duration;
  double ** duration_array = createDurationArray(req.durations, max_duration);
  Scheduler scheduler(&tasks, duration_array, max_duration);

  ROS_INFO_STREAM("SCHEDULER: Going to solve");   


   int sch_result = scheduler.solve(scheduler_version, output_file, timeout);
   if(sch_result == 1) 
   {
     ROS_INFO("SCHEDULER: found a solution");
     std::sort(tasks.begin(), tasks.end(), compareTasks);

     // clean up duration matrix... yuck! Lenka, smart pointers or, ideally, boost matrix
     clean_matrix(duration_array, tasks.size());

     for(auto & tp : tasks) 
     {
       res.task_order.push_back(tp->getID());
       res.execution_times.push_back(ros::Time(tp->getExecTime()));
       delete tp;
     } 

   }
  else if(sch_result==0) {
    ROS_INFO("SCHEDULER: didnt find a solution even though everything was fine");

    clean_matrix(duration_array, tasks.size());

    // manage memory explicitly until Lenka changes to smart pointers
    for(auto & tp : tasks) {
      delete tp;
    } 
  }
  else if(sch_result == -1)
  {
    ROS_INFO("SCHEDULER: some scip error occured\n"); 
    clean_matrix(duration_array, tasks.size());
    for(auto & tp : tasks) {
      delete tp;
    } 
  }
  else if(sch_result == -2)
  {
    ROS_INFO("SCHEDULER: The input problem had a flaw, scheduler didnt attempt\n"); 
    clean_matrix(duration_array, tasks.size());
    for(auto & tp : tasks) {
      delete tp;
    } 
  }
  else if(sch_result == -3)
  {
    ROS_WARN("SCHEDULER: created a flaw in pair assigning, scheduler didnt attempt\n"); 
    clean_matrix(duration_array, tasks.size());
    for(auto & tp : tasks) {
      delete tp;
    } 
  }

  
  return true;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "schedule_server");

  if(ros::param::has("~save_problems")) {
    ros::param::get("~save_problems", save_problems);
  } 

  if(ros::param::has("~scheduler_version")) {
    ros::param::get("~scheduler_version", scheduler_version);
    ROS_INFO_STREAM("SCHEDULER: Running scheduler version " << scheduler_version);
  } 

  if(ros::param::has("~output_file")) {
    ros::param::get("~output_file", output_file);
    ROS_INFO_STREAM("SCHEDULER: Saving experimental output to " << output_file);
  } 

  if(ros::param::has("~timeout")) {
    ros::param::get("~timeout", timeout);
    ROS_INFO_STREAM("SCHEDULER: Using timeout " << timeout);
  } 

  if(save_problems) {
    ROS_INFO("SCHEDULER: Writing scheduling problems to mongodb_store");
  }
  else{
    ROS_INFO("SCHEDULER: Not writing scheduling problems to mongodb_store");
  }

  	ros::NodeHandle nh;

  	ros::ServiceServer service = nh.advertiseService("get_schedule", getSchedule);
  	ROS_INFO("Ready to serve schedules");
  	ros::spin();

	return 0;
}
