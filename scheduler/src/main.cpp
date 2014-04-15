#include <iostream>
#include <string>
#include <vector>

#include "task.h"
#include "scheduler.h"

using namespace std;

int main (int argc, char** argv) 
{   
  string s1 = "home";
  string s2 = "office";
  Task a(1,0.0, 50.0, 4.0, s1, s2);
  Task b(2,0.0, 50.0, 3.0, s2, s1);
  Task c(3,0.0, 50.0, 2.0, "school", "shop",true);

  vector<Task*> tasks;

  vector<Task*>::iterator it;
  it = tasks.begin();
  tasks.insert(it,&a);
  it = tasks.begin()+1;
  tasks.insert(it,&b);
  it = tasks.begin()+2;
  tasks.insert(it,&c);

  Task d(4,0.0, 50.0, 1.0, "shop", "school");//,&tasks);
  
  vector<Task*> schedT;
  it = schedT.begin();
  schedT.insert(it,&a);
  it = schedT.begin()+1;
  schedT.insert(it,&b);
  it = schedT.begin()+2;
  schedT.insert(it,&c);
  it = schedT.begin()+3;
  schedT.insert(it,&d);
 

  Scheduler sch(&schedT);
  bool worked = sch.solve();

  cout<< "Schedule found" << worked << "\n";
  for(int i=0;i<4;i++)
  {
    cout<< schedT[i]->getExecTime() << "\n";
  }

  return 0;

}
