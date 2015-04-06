#include "ros/ros.h"

#include <iostream>
#include <string>
#include <vector>
#include <ctime>

#include "task.h"
#include "scheduler.h"
//#include "priorities.h"

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




  /*for (int i = 0; i < task_count; ++i)
  {
    if((i==5)||(i==6))
      tasks.push_back(new Task(i, window_start, window_end, task_duration, s1, s2, 10));
    else
      tasks.push_back(new Task(i, window_start, window_end, task_duration, s1, s2, 1));
    //window_start = window_end + 10;
    //window_end = window_start + (task_duration * 2);
  }*/

  tasks.push_back(new Task(1, 1428069688.704307079, 1428069694.704307079, 5, s1, s2, 1));
  tasks.push_back(new Task(1, 1428069688.704307079, 1428069694.704307079, 5, s1, s2, 1));
  tasks.push_back(new Task(1, 1428069688.704307079, 1428069694.704307079, 5, s1, s2, 1));

  
     
  //TODO this doesnt work properly
  //double ** duration_array = toArray(createUniformDurationMatrix(&tasks, 1.0));
  double ** duration_array = new double*[3];
  for (int i = 0; i<3; i++)
    duration_array[i] = new double[3];

  duration_array[0][ 0] = 0;
  duration_array[0][ 1] = 10;
  duration_array[0][ 2] = 20;
  duration_array[1][ 0] = 20;
  duration_array[1][ 1] = 0;
  duration_array[1][ 2] = 10;
  duration_array[2][ 0] = 10;
  duration_array[2][ 1] = 20;
  duration_array[2][ 2] = 0;

  vector<Task*> * rest = new vector<Task*>();
  vector<Task*> * result = new vector<Task*>();

  cout << "in main" << duration_array[0][1];
  //Priorities priorities(&tasks, duration_array, 1.0);
  //priorities.getSubset(result, rest);
  
  

  Scheduler scheduler(&tasks, duration_array, 1.0);
  int worked = scheduler.solve(4,filename);

  /*if(worked)  {
    cout<< "Schedule found" << worked << "\n";
    for(auto & tp : tasks) {
      cout<< "Is start time "<<tp->getExecTime() << " after start? "
       << (tp->getExecTime() >= tp->getStart()) << "\n";
    } 
  }
  else {
    cout<< "No schedule found"<< endl;
  }
*/
  

  return 0;

}
