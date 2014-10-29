//------------------------------
//guarding to avoid multiple including
#ifndef __SCHEDULER_H_INCLUDED__
#define __SCHEDULER_H_INCLUDED__

#include <string>
//-----------------------------
// forward declared dependencies (I am using pointer to the Task, I dont need include it)
class Task;
class ScipUser;
class Pairs;

using namespace std;

class Scheduler
{
  vector<Task *> * tasksToS; //vector of tasks to be schedule
  int numPairs;
  int numTasks;
  vector<vector<int>> pairs;
  double ** dist_a;
  public:
  Scheduler(vector<Task *>*);
  double getMaxDist();
  double readDist();
  int setPreVar(ScipUser *);
  int findTaskNow();
  vector<int> findConditions();
  bool solve(int, string, const int & timeout = 0);
  
  
};

#endif
