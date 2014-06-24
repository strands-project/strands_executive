#include <vector>
#include "scheduler.h"
#include "task.h"
#include "distWrapper.h"
#include "pairs.h"
#include <iostream> //TODO: delete
#include "scipUser.h"
#include <math.h>  
#include <chrono>
#include <fstream>
/* scip includes */
#include "objscip/objscip.h"
#include "objscip/objscipdefplugins.h"

using namespace scip;

using namespace std;

Scheduler::Scheduler(vector<Task *> * tasks)
{
  tasksToS = tasks;
  numTasks = tasks->size();
  numPairs = 0;
}

int Scheduler::getNumPairs()
{
  return numPairs;
}

int Scheduler::getNumTasks()
{
  return numTasks;
}

vector<vector<int>> Scheduler::getPairs()
{
  return pairs; //TODO: think if this is good, maybe return reference to it?
}

double Scheduler::getMaxDist()
{
  double max=0;
  double dist;
  for(int i=0; i< numTasks; i++)
  {
    for(int j=0; j< numTasks; j++)
    {
      if(i != j)
      {
        dist = DistWrapper::dist(tasksToS->at(i)->getEndPos(),tasksToS->at(j)->getStartPos());
        if(dist>max)
        {  
          max = dist; 
        }
      }
    }
  }
  return max;
}


int Scheduler::findTaskNow()
{
  int index = -1;
  for(int i=0; i<(int)tasksToS->size();i++)
  {
    Task * ac = tasksToS->at(i);
    if (ac->getNow())
    {
      index = i;
      break;
    }      
  }
  return index;
}

vector<int> Scheduler::findConditions()
{
  vector<int> con;
  vector<int>::iterator it;
  int vi=0;
  for(int i=0; i<(int)tasksToS->size();i++)
  {
    Task * ac = tasksToS->at(i);
    if (ac->getCond())
    {
      it = con.begin() + vi;
      con.insert(it,i);
      vi++;
    }
  }
  return con;
}

int Scheduler::setPreVar(ScipUser * solver)
{
  vector<bool> pairSet;
  pairSet.resize(numPairs,false);
  //int i;
  int j;
  int * order = new int();
  //setting pre variables, first testing now and conditions
  //if task now exist, we need to set pre variables first

  /*i = findTaskNow();
  if(i != -1)
  {
    unsigned int tid = tasksToS->at(i)->getID();
    vector<bool> nowSet; //vector indicating if all pairs were set
    nowSet.resize(tasksToS->size(),false);
    //for task now we need to create pairs with all tasks (we need to ensure, that task now will precede all of them
    for(int k=0; k<(int)tasksToS->size();k++)
    {
      if(i != k) //for all tasks instead of the tasks now, we dont want to crate pair i,i
      {

        unsigned int kid = tasksToS->at(k)->getID();
      //go throw pairs 
        for (j=0; j<(int)pairs.size();j++)
        {
          vector<int> p = pairs.at(j);
          //if the task is in pair on first place, we would like to fix pre_ij = 1 (task i will precede task on the second place)
          if((tasksToS->at(p.at(0))->getID() == tid)&&(tasksToS->at(p.at(1))->getID() == kid))
          {
            pairSet[j] = true;
            nowSet[k] = true;
            pairs.at(j).at(3) = 1;
            pairs.at(j).at(2) = *order;
       
          }
          //if the task is in pair on the second place, it means that variable pre_ij must be zero, because then pre_ji is one
          else if((tasksToS->at(p.at(1))->getID() == tid)&&(tasksToS->at(p.at(0))->getID() == kid))
          { 
            pairSet[j] = true;
            nowSet[k] = true;
            pairs.at(j).at(3) = 0;
            pairs.at(j).at(2) = *order;
          }
        }
        if(!nowSet[k]) //if pair doesnt exist in pairs, we need to set it
        {
           nowSet[k] = true;
           vector<vector<int>>::iterator it;
           it = pairs.begin()+numPairs;
           vector<int> opair(4);
           //to have pair always with smaller number first
           if(i < k)
           {
             opair[0] = i;//tid;
             opair[1] = k;//kid;
             opair[2] = *order;
             opair[3] = 1;
           }
           else
           {
             opair[0] = k;//kid;
             opair[1] = i;//tid;
             opair[2] = *order;
             opair[3] = 0;
           }
           //also we need to add it to the pairs, to be able to create constraints;
           

           pairs.insert(it,opair); 
           vector<bool>::iterator pit;
           pit = pairSet.begin()+numPairs;
           pairSet.insert(pit,true);
           numPairs++;  
        }
      }
    }   
  }
  
  //then solve conditions
  vector<int> taskWithCond = findConditions();
  if(taskWithCond.size() != 0)
  {
    //for tasks witch have preconditions
    for(i =0; i<(int)taskWithCond.size(); i++)
    { 
      Task * ac = tasksToS->at(taskWithCond.at(i));

      unsigned int idt = ac -> getID();
      vector<Task*> * precon = ac->getPrecon();
      vector<bool> preSet;
      preSet.resize(precon->size(),false); //for each precondition we need to set some variable
      //for all preconditions for single task
      for(int k=0; k<(int)precon->size(); k++)
      {
        unsigned int idp = precon->at(k)->getID();
        //set possible pairs
        for (j=0; j<(int)pairs.size();j++)
        {
          vector<int> p = pairs.at(j);
          //if i,j is in pair
          if((tasksToS->at(p.at(0))->getID() == idt)&&(tasksToS->at(p.at(1))->getID() == idp))
          { 
            if(!pairSet[j])
            {
              //i is the task which has j as precondition, thus preij = 0
              pairSet[j] = true;
              preSet[k] = true;
              //err = solver->preVar(idt, idp, 0.0, 0.0,order); //seting low and up to same value will fix it to that
              pairs.at(j).at(3) = 0;
              pairs.at(j).at(2) = *order;
            }
            else
            { //if pair already set, notify that the conditions is solved
              preSet[k] = true;
            }
          }
          //if j,i is in pair
          else if((tasksToS->at(p.at(0))->getID() == idp)&&(tasksToS->at(p.at(1))->getID() == idt))
          { 
            if(!pairSet[j])
            {
              //i is the task which has j as precondition, thus preij = 1
              pairSet[j] = true;
              preSet[k] = true;
              //err = solver->preVar(idp, idt, 1.0, 1.0,order); //seting low and up to same value will fix it to that
              pairs.at(j).at(3) = 1;
              pairs.at(j).at(2) = *order;
            }
            else
            { //if pair already set, notify that the conditions is solved
              preSet[k] = true;
            }
          }
        }
 
        //if the pair does not exists (because tasks are not overlapping) condition for precondition k was not established
        if(!preSet[k])
        {
           preSet[k] = true;
           vector<vector<int>>::iterator it;
           it = pairs.begin()+numPairs;
           vector<int> opair(4);

           if(taskWithCond.at(i) < k)
           {
             //err = solver->preVar(idt, idp, 0.0, 0.0,order); //seting low and up to same value will fix it to that
             opair[0] = taskWithCond.at(i);//idp;
             opair[1] = k;//idt;
             opair[2] = *order;
             opair[3] = 0;
           }
           else
           {
             //err = solver->preVar(idp, idt, 1.0, 1.0,order); //seting low and up to same value will fix it to that
             opair[0] = k;//idt;
             opair[1] = taskWithCond.at(i);//idp;
             opair[2] = *order;
             opair[3] = 1;
           }

           //also we need to add it to the pairs, to be able to create constraints;


           pairs.insert(it,opair); 
           vector<bool>::iterator pit;
           pit = pairSet.begin()+numPairs;
           pairSet.insert(pit,true);
           numPairs++;     
      
        }
      }
    }
  }*/

  //set the rest of pairs
  for (j=0; j<(int)pairs.size();j++)
  {
    vector<int> p = pairs.at(j);
    if(!pairSet[j])
    {
      pairSet[j] = true;
      pairs.at(j).at(2) = *order;
     }
   }
    
  
  return 0;
}

bool Scheduler::solve(int version, string filename, const int & timeout)
{
  SCIP_Retcode err;
  vector<bool> pairUsed;
  ofstream results;

  ScipUser * solver = new ScipUser();
  err = solver->getEr();
  if (err != SCIP_OKAY)
    return -1;

  cout<<"Solving with: "<<version<<endl;
  std::chrono::high_resolution_clock::time_point start, end;
  Pairs * pr = new Pairs(tasksToS);
  
  start = std::chrono::high_resolution_clock::now();
  if(version==1)  {
    //Brian Coltin
    numPairs = pr->setPairs_BC();
  }
  else if(version==2) {
    //mine
    numPairs = pr->setPairs_mine();
  }
  else if(version==3) {
    //robot version
    numPairs = pr->setPairs();
  }
  else if(version==4) {
    //mine specific approach
    numPairs = pr->setPairs_new();
  }
  else {
    //if parameter is not set
    numPairs = pr->setPairs_new();
  }
  end = std::chrono::high_resolution_clock::now();
  
  std::chrono::duration<double> elapsed_seconds = end-start;
  if(!filename.empty())
  {
    results.open (filename,std::ios_base::app);
    results << elapsed_seconds.count() << " ";
    results.close();
  }

  pr->getPairs(&pairs);

//creating a vector for variables
  vector<SCIP_VAR *> * t_var = new vector<SCIP_VAR *>(numTasks,(SCIP_VAR*) NULL); 
  err = solver->tVar(numTasks,t_var);
  if (err != SCIP_OKAY)
    return -1;


  int e = setPreVar(solver);
  if (e==-1)
    return -1;


//create constraints for starting and ending time
  err = solver->setTcons(tasksToS, t_var);
  if (err != SCIP_OKAY)
    return -1; 

  double maxDist = getMaxDist();
//for all pairs we need to set condition
  start = std::chrono::high_resolution_clock::now();
  err = solver->setFinalCons(tasksToS, t_var, &pairs, maxDist);
  end = std::chrono::high_resolution_clock::now();
  if (err != SCIP_OKAY)
    return -1; 

  elapsed_seconds = end-start;
  if(!filename.empty())
  {
    results.open (filename,std::ios_base::app);
    results << elapsed_seconds.count() << " ";
    results.close();
  }

//conversion from vector to "array"
  SCIP_VAR * array_tvar[numTasks];
  for(int i=0; i<numTasks; i++)
    array_tvar[i] = t_var->at(i);

  bool * worked = new bool();


//----------------------- 

  err = solver->scipSolve(tasksToS, array_tvar, worked, filename, timeout);
   if (err != SCIP_OKAY)
    return -1; 

  //call destructor
  delete solver;

  return *worked;
}



