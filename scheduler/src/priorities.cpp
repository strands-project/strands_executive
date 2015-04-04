/**
  This class split the input vector of Tasks to seperate vectors based on their priorities.

  @author Lenka Mudrova
  @version 1.0 25/03/2015
*/

#include <vector>
#include "priorities.h"
#include "task.h"
#include <iostream> //TODO: delete
#include <algorithm>


//namespace bn = boost::numeric::ublas;

using namespace std;

bool sortPriorities(Task * i, Task * j)
{
  return i->getPriority() > j->getPriority();
}

Priorities::Priorities(std::vector<Task *>* tasks, double ** duration_matrix, double max_duration)
{
  
  numTasks = tasks->size();
  this->duration_matrix = duration_matrix;
  this->max_duration = max_duration;

  std::sort(tasks->begin(), tasks->end(), sortPriorities);
  tasksToS = tasks;

 
  if(numTasks > 0)
  {
    actual_prio = tasksToS->at(0)->getPriority();
    actual_index = 0;
  }
  else
  {
    actual_prio = -1;
    actual_index = -1;
  }    
}


void Priorities::getSubset(std::vector<Task *>* result, std::vector<Task *>* rest)
{
  //result->resize(numTasks);
  //rest->resize(numTasks);
  

  for(int i =actual_index; i<numTasks; i++)
  {
    if(tasksToS->at(i)->getPriority() == actual_prio)
    {
      result->push_back(tasksToS->at(i));
    }
    else
    {
      actual_prio = tasksToS->at(i)->getPriority();
      actual_index = i;
      break;
    }
  }
  for(int i = actual_index; i< numTasks;i++)
  {
    rest -> push_back(tasksToS->at(i));
  }
}



