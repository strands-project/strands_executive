/**
  This class initialise the scheduling problem and call the SCIP solver to solve it.

  @author Lenka Mudrova
  @version 2.0 04/04/2015
*/

#include <vector>
#include "scheduler.h"
#include "task.h"
#include "pairs.h"
#include <iostream> //TODO: delete
#include "scipUser.h"
#include <math.h>  
#include <chrono>
#include <fstream>
#include <algorithm> 
/* scip includes */
#include "objscip/objscip.h"
#include "objscip/objscipdefplugins.h"

using namespace scip;

using namespace std;

namespace bn = boost::numeric::ublas;


bn::matrix<double> createUniformDurationMatrix(vector<Task *>* tasks, double dist) {

  bn::matrix<double> dm(tasks->size(), tasks->size());

  for (unsigned i = 0; i < tasks->size(); ++i) {
    for (unsigned j = 0; j < tasks->size(); ++j) {
      if(i == j) {
        dm(i, j) = 0;
      }
      else {
        dm(i, j) = dist;
      }
    }
  }

  return dm;

}



/** 
  Constructor - initialisation of the scheduling problem. It also sets an array of distance for later usage. 
  @param - vector of pointers to tasks, which should be schedule
  @return - nan, it is constructor
*/
Scheduler::Scheduler(vector<Task *> * tasks, double ** duration_matrix, double max_duration)
{
  tasksToS = tasks;
  numTasks = tasks->size();
  numPairs = 0;
  this->duration_matrix = duration_matrix;
  this->max_duration = max_duration;
  
}

/**
  Desctructor
*/
Scheduler::~Scheduler()
{
  tasksToS = NULL;
  numTasks = 0;
  numPairs = 0;
  duration_matrix = NULL; //but the matrix still exist in system, must be cleaned how call scheduler!
  max_duration = 0;
}



/**
  This is the main method for a scheduler, which calls the rest

  @param version = 1 - Brian Coltin original work. Mainly for experiments
         version = 4 - Mine approach
         filename - if given, some times needed to computation are saved
         timeout - if given, the SCIP solver will be killed after this time is reached
  @return 1 if the SCIP finds a solution 
          0 if solver didnt find a solution but anything else was fine
         -1 if there is a problem with scip
         -2 if pairs detected flaw in input data
         -3 if pairs created flaw themselves - the order of tasks needs to be changed or one task throw away...
*/
int Scheduler::solve(int version, string filename, const int & timeout)
{
  //scheduler save to file: 
  //preprocessing time to create pairs, setting the constraints, infeas, objective criteria, time to solve, number of tasks, if it worked
  SCIP_Retcode err;
  std::chrono::high_resolution_clock::time_point start, end;
 

  //uncomment here to save tasks to the file, mainly debugging purpose
  /*ofstream results;
  if(!filename.empty())
  {
    string name = filename + "tasks.txt";
    results.open (name,std::ios_base::app);
    results << "BATCH\n";
    for(int a =0; a< tasksToS->size(); a++)
    {
      results << tasksToS->at(a)-> getStart() << " " << tasksToS ->at(a) -> getEnd() << " " << tasksToS ->at(a) -> getDuration() << "\n"; 
    }
    results << "----------------------------------------------\n";
    results.close();
  }*/


  ScipUser * solver = new ScipUser();
  err = solver->getEr();
  if (err != SCIP_OKAY)
    return -1;

  

  //--------------SETTING PAIRS-----------------------------
 
  Pairs * pr = new Pairs(tasksToS);
  
  if(version==1)  //Brian Coltin
    numPairs = pr->setPairs_BC();
  else //mine specific approach, should use number 4, but someone might use old number 
    numPairs = pr->setPairs_new(duration_matrix);
 
  if(numPairs == -1) //flaw in pairs occured
    return -2;
  else if(numPairs == -2) //created flaw
    return -3;

  //pairs are fine we can continue
  pr->getPairs(&pairs);

  //--------------CREATING T VARIABLES AND CONSTRAINTS s<=t & t+d <=e ---------------- (t = execution time)
  vector<SCIP_VAR *> * t_var = new vector<SCIP_VAR *>(numTasks,(SCIP_VAR*) NULL); 
  err = solver->tVar(numTasks,t_var);
  if (err != SCIP_OKAY)
    return -1;

  vector<SCIP_CONS *> * t_con = new vector<SCIP_CONS *>(numTasks,(SCIP_CONS *) NULL); 
  err = solver->setTcons(tasksToS, t_var, t_con);
  if (err != SCIP_OKAY)
    return -1; 


  //----------SETTING PAIRS CONDITIONS ----------for all pairs we need to set condition, that no tasks are overlapping
  if(version != 1) //my
  {
    err = solver->setFinalCons_new(tasksToS, t_var, &pairs, max_duration, filename, t_con, duration_matrix);
  }
  else //Brian
  {
    err = solver->setFinalCons_preVar(tasksToS, t_var, &pairs, max_duration, duration_matrix);
  }
  if (err != SCIP_OKAY)
    return -1; 


  //-------------SOLVING------------------
  //conversion from vector to "array"
  SCIP_VAR * array_tvar[numTasks];
  for(int i=0; i<numTasks; i++)
    array_tvar[i] = t_var->at(i);

  bool * worked = new bool();

  //calling solver for the formulated MIP problem
  err = solver->scipSolve(tasksToS, array_tvar, worked, filename, timeout);
   if (err != SCIP_OKAY)
    return -1; 

  //call destructor for SCIP problem and pairs
  delete solver;
  delete pr;
 
  return (int)*worked;
}



