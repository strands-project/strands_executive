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
  double duration; 
  string s_pos; //position where the task starts
  string e_pos; //position where the task ends
  bool no;     //if the task must happen now
  bool cond;   //if the task has preceding tasks
  vector<Task*> * precon; //tasks which needs to happen before this task
  double exec_time;

  public:
  Task (unsigned int, double, double, double, string, string);
  Task (unsigned int, double, double, double, string, string,bool);
  Task (unsigned int, double, double, double, string, string,vector<Task*> *);
  unsigned int getID();
  double getStart();
  double getEnd();
  double getDuration();
  string getStartPos();
  string getEndPos();
  bool getNow();
  bool getCond();
  vector<Task*> * getPrecon();
  double getExecTime();
  void setExecTime(double);
  friend ostream& operator<<(ostream&, const Task&);
};

#endif

