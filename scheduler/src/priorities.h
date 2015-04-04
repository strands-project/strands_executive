//------------------------------
//guarding to avoid multiple including
#ifndef __PRIORITIES_H_INCLUDED__
#define __PRIORITIES_H_INCLUDED__

#include <string>
//-----------------------------
// forward declared dependencies (I am using pointer to the Task, I dont need include it)
class Task;

class Priorities
{
  std::vector<Task *> * tasksToS; //std::vector of tasks to be schedule
  int numTasks;
  double ** duration_matrix;
  double max_duration;
  std::vector<Task *> * rest; //rest to be scheduled later
  int actual_prio;
  int actual_index;

  public:
  Priorities(std::vector<Task *>* tasks, double ** duration_matrix, double max_duration);
  void getSubset(std::vector<Task *>*, std::vector<Task *>*);
  
  
  
  
};

#endif
