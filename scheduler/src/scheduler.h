//------------------------------
//guarding to avoid multiple including
#ifndef __SCHEDULER_H_INCLUDED__
#define __SCHEDULER_H_INCLUDED__

//include for duration matrix
#include <boost/numeric/ublas/matrix.hpp>

#include <string>
//-----------------------------
// forward declared dependencies (I am using pointer to the Task, I dont need include it)
class Task;
class ScipUser;
class Pairs;

namespace bn = boost::numeric::ublas;

bn::matrix<double> createUniformDurationMatrix(std::vector<Task *>* tasks, double duration = 1.0);

template <class T>
T ** toArray(const bn::matrix<T> &m) {
  
  T** array = new T*[m.size1()];
  for(int i = 0; i < m.size1(); ++i) {
    array[i] = new T[m.size2()];
  }
  return array;
}

class Scheduler
{
  std::vector<Task *> * tasksToS; //std::vector of tasks to be schedule
  int numPairs;
  int numTasks;
  std::vector<std::vector<int>> pairs;
  double ** duration_matrix;
  double max_duration;

  public:
  Scheduler(std::vector<Task *>* tasks, double ** duration_matrix, double max_duration);
  ~Scheduler();
  int solve(int, std::string, const int & timeout = 0);
  
  
};

#endif
