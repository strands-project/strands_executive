#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include "task.h"
#include "scheduler.h"
#include <fstream>

using namespace std;

int main (int argc, char** argv) 
{   
  string s1("home");
  string s2("office");
  string filename="";
  ofstream myfile;
  int version = 4;
  unsigned int task_count = 10;
  int trials = 1;

  //input parameters are assumed to be in order:
  //version - always need to be chosen
  //amount of tasks to generate
  //amount of sets
  //path to file if you want to save set
  if(argc == 1)
  {
    cout << "You need to set these parameters! \n" << "version of algorithm [1,4] - always need to be chosen \n" <<
     "amount of tasks to be generated \n" << "amount of schedules to be created \n" << "path to file if you want to save set \n";
    return 0;
  }
  else if(argc == 2) {
    version = stoi(argv[1]);
  }
  else if(argc==3)
  {
    version = stoi(argv[1]);
    task_count = stoi(argv[2]);
  }
  else if(argc==4)
  {
    version = stoi(argv[1]);
    task_count = stoi(argv[2]);
    trials = stoi(argv[3]);
  }
  else if(argc==5)
  {
    version = stoi(argv[1]);
    task_count = stoi(argv[2]);
    trials = stoi(argv[3]);
    filename = argv[4]; //possibility of saving generated tasks to a file to later replay
  }


  
  int num_prob2 =0; //counting of how many problems couldnt be solved

  if(!filename.empty())
  {
    myfile.open (filename);
    myfile << "start, end, duration, assigned time\n";
  }
  srand(time(NULL));

  int mind = 2; //minimal duration of tasks
  int maxd = 30;  //maximal duration of tasks
  int maxw = 20; //maximum waiting time between two tasks
  int maxs = 240; //maximum window size


  //loop for generating more sets
  for(int x=0; x<trials; x++) 
  {

    int taken=0; //time taken from start of the tasks to end of its execution
    int d=0; //duration of task
    int w=0; //time when a task should be executed
    int s; //starting time of task
    int e; //ending time of task
		
    int sum=0;
  
    vector<Task*> tasks;


    for(int i=0;i<task_count;i++)
    {			
      w =w+d+(int) round(rand() % maxw) +1;
      s = w-(int) round(rand() % (maxw*5)); //set random start of task
      if(s<0)
        s=0;
      d = (int) round(rand() % (maxd-mind)+mind);
      taken=w-s + d;
      e=(int) round(rand() % (maxs))+taken+s+1;
      sum=sum+w;
      if(!filename.empty())
        myfile << s << ";" << e << ";" << d << ";" << w << "\n";

      tasks.push_back(new Task(i, s, e, d, s1, s2));
    }
    
    if(!filename.empty())
      myfile << "\n";

    Scheduler scheduler(&tasks);
    bool worked = scheduler.solve(version,""); 

    if(worked)  {
      cout<< "Schedule found" << worked << "\n";
    }
    else {
      cout<< "No schedule found"<< endl;
      num_prob2++;
    }
  }
  if(!filename.empty())
    myfile.close();

  cout << "Problems:" << num_prob2 << "\n";
  return 0;

}
