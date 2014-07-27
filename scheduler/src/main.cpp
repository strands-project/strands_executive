#include "ros/ros.h"

#include <iostream>
#include <string>
#include <vector>
#include <ctime>

#include "task.h"
#include "scheduler.h"

using namespace std;

int main (int argc, char** argv) 
{   
  ros::init(argc, argv, "error_test");
  string s1("home");
  string s2("office");

  unsigned int task_count;

  //getting time stamp
  time_t now = time(0);
  tm *ltm = localtime(&now);  
  string filename = "../result/real" + to_string(ltm->tm_hour)+"_"+to_string(ltm->tm_min)+"_"+to_string(ltm->tm_sec) + ".csv";

  if(argc == 1)
  {  
    task_count = 10; 
  }
  else if(argc == 2) {
    task_count = stoi(argv[1]);
  }
  else {
    task_count = stoi(argv[1]);
    filename=argv[2];   
  }

  double one_hour = 60 * 6;
  double task_duration = one_hour / 2.0;

  double window_start = 0;
  double window_end = window_start + (task_duration * task_count * task_count);

  vector<Task*> tasks;

cout << "this is a test!";

  for (int i = 0; i < task_count; ++i)
  {
    
    tasks.push_back(new Task(i, window_start, window_end, task_duration, s1, s2));
    //window_start = window_end + 10;
    //window_end = window_start + (task_duration * 2);
  }


  Scheduler scheduler(&tasks);
  bool worked = scheduler.solve(2,filename);

  if(worked)  {
    cout<< "Schedule found" << worked << "\n";
    for(auto & tp : tasks) {
      cout<< "Is start time "<<tp->getExecTime() << " after start? "
       << (tp->getExecTime() >= tp->getStart()) << "\n";
    } 
  }
  else {
    cout<< "No schedule found"<< endl;
  }

  return 0;

}
