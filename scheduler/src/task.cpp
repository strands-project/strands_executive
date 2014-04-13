#include <string>
#include <iostream> 
#include <vector>
#include "task.h"

using namespace std;

/*constructor without parameter now, automatically set now to false*/
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
}

/*constructor with parameter now*/
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
}

/*constructor without parameter now, automatically set now to false and with vector of tasks, which needs to precede this task*/
Task::Task(unsigned int ID, double s, double e, double d, string start_pos, string end_pos, vector<Task*> * pre)
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
}
//TODO: think if task with "now" needs to have also preconditions, right now it doesnt

unsigned int Task::getID() {return id;}
double Task::getStart() {return start;}
double Task::getEnd() {return end;}
double Task::getDuration() {return duration;}
string Task::getStartPos() {return s_pos;}
string Task::getEndPos() {return e_pos;}
bool Task::getNow() {return no;}
bool Task::getCond() {return cond;}
vector<Task*> * Task::getPrecon() {return precon;}
double Task::getExecTime() {return exec_time;}
void Task::setExecTime(double execTime) {exec_time = execTime;}

std::ostream& operator<<(std::ostream& os, const Task& t)
{
  if(t.no)
  {
    os << "[" << t.id << t.start << "," <<t.end << "," << t.duration << "," <<t.s_pos<< "," << t.e_pos << "," << "now" << "]";
  }
  else
  {
    if(t.cond)
    {
      os << "[" << t.id << t.start << "," <<t.end << "," << t.duration << "," <<t.s_pos<< "," << t.e_pos  << ",\n";
      for(unsigned int i=0; i< t.precon->size();i++)
      {
        Task * x = t.precon->at(i);
        os << "prec:"<<*x << ",\n";
      }
      os <<  "]";
    }
    else
    {
      os << "[" << t.id << t.start << "," <<t.end << "," << t.duration << "," <<t.s_pos<< "," << t.e_pos  << "]";
    }
  }
  
  
  return os;
}

