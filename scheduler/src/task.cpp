/**
  definition of a task

  @author Lenka Mudrova
  @version 2.0 15/03/2015
*/

#include <string>
#include <iostream> 
#include <vector>
#include "task.h"

using namespace std;

/** 
  constructor without parameter now, automatically set now to false
  @param ID of the task, release time, deadline, processing time, start position, end position
  @return nan, it is constructor
*/
Task::Task(unsigned int ID, double s, double e, double d, string start_pos, string end_pos)
{
  id = ID;
  start = s;
  end = e;
  duration = d;
  s_pos = start_pos;
  e_pos = end_pos;
  no = false;
  cond = false;
  priority = 1;
}

/** 
  constructor without parameter now, automatically set now to false
  @param ID of the task, release time, deadline, processing time, start position, end position, priority
  @return nan, it is constructor
*/
Task::Task(unsigned int ID, double s, double e, double d, string start_pos, string end_pos, int prio)
{
  id = ID;
  start = s;
  end = e;
  duration = d;
  s_pos = start_pos;
  e_pos = end_pos;
  no = false;
  cond = false;
  priority = prio;
}


/**
  constructor with parameter now
  @param ID of the task, release time, deadline, processing time, start position, end position, now flag
  @return nan, it is constructor
*/
Task::Task(unsigned int ID, double s, double e, double d, string start_pos, string end_pos,bool now)
{
  id = ID;
  start = s;
  end = e;
  duration = d;
  s_pos = start_pos;
  e_pos = end_pos;
  no = now;
  cond = false;
  priority = 1;
}

/** 
  constructor without parameter now, automatically set now to false and with vector of tasks, which needs to precede this task
  @param ID of the task, release time, deadline, processing time, start position, vector of pointers to preceding tasks
  @return nan, it is constructor
*/
Task::Task(unsigned int ID, double s, double e, double d, string start_pos, string end_pos, vector<Task*> * pre, int prio)
{
  id = ID;
  start = s;
  end = e;
  duration = d;
  s_pos = start_pos;
  e_pos = end_pos;
  no = false;
  precon = pre;
  cond = true;
  priority = prio;
}

unsigned int Task::getID() {return id;}
double Task::getStart() {return start;}
double Task::getEnd() {return end;}
void Task::setEnd(double new_end){end = new_end;}
double Task::getDuration() {return duration;}
string Task::getStartPos() {return s_pos;}
string Task::getEndPos() {return e_pos;}
bool Task::getNow() {return no;}
bool Task::getCond() {return cond;}
vector<Task*> * Task::getPrecon() {return precon;}
double Task::getExecTime() {return exec_time;}
void Task::setExecTime(double execTime) {exec_time = execTime;}
int Task::getPriority() {return priority;}
void Task::setPriority(int prio) {priority = prio;}

std::ostream& operator<<(std::ostream& os, const Task& t)
{
  if(t.no)
  {
    os << "[" << t.id <<"," << t.start << "," <<t.end << "," << t.duration << "," <<t.s_pos<< "," << t.e_pos << "," << "now" << "," << t.priority << "]";
  }
  else
  {
    if(t.cond)
    {
      os << "[" << t.id <<"," << t.start << "," <<t.end << "," << t.duration << "," <<t.s_pos<< "," << t.e_pos  << "," << t.priority << ",\n";
      for(unsigned int i=0; i< t.precon->size();i++)
      {
        Task * x = t.precon->at(i);
        os << "prec:"<<*x << ",\n";
      }
      os <<  "]";
    }
    else
    {
      os << "[" << t.id <<"," << t.start << "," <<t.end << "," << t.duration << "," <<t.s_pos<< "," << t.e_pos  << "," << t.priority << "]";
    }
  }
  
  
  return os;
}

