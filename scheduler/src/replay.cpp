#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include "task.h"
#include "scheduler.h"
#include <fstream>
#include "ros/ros.h"

using namespace std;

int main (int argc, char** argv)
{
  string s1("home");
  string s2("office");

  int num_prob1 =0;
  int num_prob2 =0;
  ifstream myfile;
  string line;
  string name_file;
  int s,e,d;
  int num=0;
  int task_count;
  int version;	

  ros::init(argc, argv, "replay");

  vector<Task*> tasks;

  if(argc > 3) {
    task_count = stoi(argv[1]);
    name_file = argv[2];
    version = stoi(argv[3]);
  }
  else 
  {
    cout << "wrong arguments";
    return 0;
  }

  myfile.open ("/home/lenka/phd/text/Articles/ICRA2015new/Data/moreoverlapping/"+name_file, std::ios_base::in);

  if (myfile.is_open())
  {

    while (myfile >> s >> e >> d )
    {
      cout << s <<";" << e <<";" << d << "\n";
      tasks.push_back(new Task(num+1, s, e, d, s1, s2));
      num++;
      if(num%task_count == 0)
      {
        Scheduler scheduler(&tasks);
        line= "/home/lenka/phd/text/Articles/ICRA2015new/Data/"+to_string(task_count)+"version"+ to_string(version)+".txt";
        bool worked = scheduler.solve(version,line,3*60);
        tasks.erase(tasks.begin(),tasks.end());
        cout << "zkouska" << tasks.size() << "\n";
        if(worked) {
          cout<< "Schedule found" << worked << "\n";
         
        }
        else {
          cout<< "No schedule found"<< endl;
          num_prob2++;
        }
      }
    }
    myfile.close();
  }
  
  cout << "Problems:" << num_prob1 << ";" << num_prob2 << "\n";
  return 0;

}
