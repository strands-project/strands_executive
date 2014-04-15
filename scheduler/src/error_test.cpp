#include <iostream>
#include <string>
#include <vector>

#include "task.h"
#include "scheduler.h"

using namespace std;

int main (int argc, char** argv) 
{   
  string s1("home");
  string s2("office");
  double start_after = 10;
  double end_before = 50;
  Task a(1, start_after, end_before, 4.0, s1, s2);
  // Task b(2, start_after, end_before, 3.0, s2, s1);
  
  vector<Task*> tasks;

  tasks.push_back(&a);
  // tasks.push_back(&b); 

  Scheduler scheduler(&tasks);
  bool worked = scheduler.solve();

  cout<< "Schedule found" << worked << "\n";
  for(auto & tp : tasks) {
    cout<< "Is start time "<<tp->getExecTime() << " after start? "
     << (tp->getExecTime() >= tp->getStart()) << "\n";
  }

  return 0;

}
