//------------------------------
//guarding to avoid multiple including
#ifndef __TASK_H_INCLUDED__
#define __TASK_H_INCLUDED__

#include <string>
#include <vector>

using namespace std;
class Task
{
  unsigned int id; //a number refering to the task 
  double start;  //the time when the task can start
  double end;    //the time when the task needs to be finished
  double duration; //task duration, ignoring travel 
  string s_pos; //position where the task starts
  string e_pos; //position where the task ends
  bool no;     //if the task must happen now
  bool cond;   //if the task has preceding tasks
  vector<Task*> * precon; //tasks which needs to happen before this task
  double exec_time;
  int priority; //this is propage to minimisation criteria, when task's completion time is penalised by this number.

  public:
  Task (unsigned int, double, double, double, string, string); //default priority = 1
  Task (unsigned int, double, double, double, string, string, int); //support of user priority
  Task (unsigned int, double, double, double, string, string,bool); //on demand task doesnt have to have priority as it is executed immediately
  Task (unsigned int, double, double, double, string, string,vector<Task*> *, int); //task and it preceding tasks
  unsigned int getID();
  double getStart();
  double getEnd();
  void setEnd(double);
  double getDuration();
  string getStartPos();
  string getEndPos();
  bool getNow();
  bool getCond();
  vector<Task*> * getPrecon();
  double getExecTime();
  void setExecTime(double);
  int getPriority();
  void setPriority(int);
  friend ostream& operator<<(ostream&, const Task&);
};

#endif

