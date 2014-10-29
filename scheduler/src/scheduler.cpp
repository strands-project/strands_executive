/**
  This class initialise the scheduling problem and call the SCIP solver to solve it.

  @author Lenka Mudrova
  @version 1.0 29/10/2014
*/

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
#include <algorithm> 
/* scip includes */
#include "objscip/objscip.h"
#include "objscip/objscipdefplugins.h"

using namespace scip;

using namespace std;


/** 
  Constructor - initialisation of the scheduling problem. It also sets an array of distance for later usage. 
  @param - vector of pointers to tasks, which should be schedule
  @return - nan, it is constructor
*/
Scheduler::Scheduler(vector<Task *> * tasks)
{
  tasksToS = tasks;
  numTasks = tasks->size();
  numPairs = 0;
  dist_a = new double*[tasks->size()];
  for(int i = 0; i < tasks->size(); i++)
    dist_a[i] = new double[tasks->size()];
}

//TODO destructor


/**
  This method initialises an array of distances between locations, which are used in a set of tasks. 
  The array is used later in order to minimise the amount of calling DistWrapper.
  It also computes max distance.
  @param none
  @return maximal distance, which occured in the system
*/
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
        dist_a[i][j] = dist;
        if(dist>max)
        {  
          max = dist; 
        }
      }
    }
  }
  return max;
}

/**
  This method is used during experiments. It reads the array of distances from the file. It is hard-coded for specific datasets G4S or AAF.
  Not for public use.
  @param none
  @return maximal distance
*/
double Scheduler::readDist() 
{
  //string filename = "/home/lenka/phd/code/strands_ws/src/strands_executive/scheduler/data/distances_aaf.txt";
  string filename = "/home/lenka/phd/code/strands_ws/src/strands_executive/scheduler/data/distances_g4s.txt";
  ifstream results;
  results.open (filename,std::ios_base::in);

  double max=0;
  double dist;
  int numWay = 46; //or 17 for aaf 46 for g4s
  double arr_dist[numWay][numWay];

  double n;
  int i=0,j=0;
  while(results >> n)
  {
    arr_dist[i][j] = n;
    j++;
    if(n>max)
        {  
          max = n; 
        }
    if(j > numWay-1)
    {
      j = 0;
      i++;
    }
  }
 results.close();

  string point[] = {"Lobby", "ChargingPoint", "OfficeDoorInside", "OfficePoint", "OfficeDoorOutside", "SafePointLobby", "SafePointCorridor", "CrossRoads", "CorridorToilets", "GamePoint", "FireDoorWest", "FireDoorEast", "ChapelDoors", "FireExtinguisher", "PrinterRoom", "GameTable", "CorridorEast"};


  for(int i=0; i< numTasks; i++)
  {
    for(int j=0; j< numTasks; j++)
    {
      if(i != j)
      {
        int x = -1,y = -1,sizex,sizey;
        string EP = tasksToS->at(i)->getEndPos();
        string SP = tasksToS->at(j)->getStartPos();

        //aaf
        /*for(int k = 0; k < numWay; k++)
        {
          if(EP == point[k])
            x = k;
          if(SP == point[k])
            y = k;
        }*/
 
        //g4s
        sizex = EP.size();
        sizey = SP.size();

        if(sizex >=8)
        {
          if(EP == "ChargingPoint")
            x = 0;
          else if(EP.substr(0,8)=="WayPoint")
          {
            EP = EP.substr(8,sizex);  
            x = stoi(EP);
          }
        }
        if(sizey >= 8)
        {
          if(SP == "ChargingPoint")
            y = 0;
          else if(SP.substr(0,8)=="WayPoint")
          {
            SP = SP.substr(8,sizey);  
            y = stoi(SP);
          }
        }

        if((x != -1)&&(y != -1))
          dist_a[i][j] = arr_dist[x][y];
        
      }
       
    }
  }
  
 
  
  return max;
}

/**
  This method returns the index of a task which has "now" flag.
  @param none
  @return index of the task
*/
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

/**
  This method returns a vector of indexes which has "precondition" flag.
  @param none
  @return vector of indexes
*/
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

/**
  This method sets pairs for those tasks, which have some special flags - now or preconditions.
  TODO needs clearing
  @param pointer to SCIP solver
  @return 0 if everything went well
*/
int Scheduler::setPreVar(ScipUser * solver)
{
  vector<bool> pairSet;
  pairSet.resize(numPairs,false);
  int i;
  int j;
  int * order = new int();
  //setting pre variables, first testing now and conditions
  //if task now exist, we need to set pre variables first

  i = findTaskNow();
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
  /*
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

/**
  This is the main method for a scheduler, which calls the rest

  @param version = 1 - Brian Coltin original work. Mainly for experiments
         version = 4 - Mine approach
         filename - if given, some times needed to computation are saved
         timeout - if given, the SCIP solver will be killed after this time is reached
  @return if the SCIP finds a solution or not
*/
bool Scheduler::solve(int version, string filename, const int & timeout)
{
  //scheduler save to file: 
  //preprocessing time to create pairs, setting the constraints, infeas, objective criteria, time to solve, number of tasks, if it worked
  SCIP_Retcode err;
  vector<bool> pairUsed;
  ofstream results;

  ScipUser * solver = new ScipUser();
  err = solver->getEr();
  if (err != SCIP_OKAY)
    return -1;

  //uncomment here to save tasks to the file, mainly debugging purpose
  /*if(!filename.empty())
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


  
  std::chrono::high_resolution_clock::time_point start, end;

  double maxDist = getMaxDist();
 
  Pairs * pr = new Pairs(tasksToS);
  
  
  if(version==1)  {
    //Brian Coltin
    numPairs = pr->setPairs_BC();
  }
  else if(version==4) {
    //mine specific approach
    numPairs = pr->setPairs_new(dist_a);
  } 
  else {
    //if parameter is not set
    numPairs = pr->setPairs_new(dist_a);
  }

  pr->getPairs(&pairs);

  //creating a vector for t variables (t = execution time)
  vector<SCIP_VAR *> * t_var = new vector<SCIP_VAR *>(numTasks,(SCIP_VAR*) NULL); 
  err = solver->tVar(numTasks,t_var);
  if (err != SCIP_OKAY)
    return -1;

  //checking if some task has "now" or "precondition" flag. If yes, pairs needs to be set in different way
  int e = setPreVar(solver);
  if (e==-1)
    return -1;


  //create constraints that holds s<= t <= e
  vector<SCIP_CONS *> * t_con = new vector<SCIP_CONS *>(numTasks,(SCIP_CONS *) NULL); 
  err = solver->setTcons(tasksToS, t_var, t_con);
  if (err != SCIP_OKAY)
    return -1; 


  //for all pairs we need to set condition, that no tasks are overlapping
  if(version != 1)
  {
    err = solver->setFinalCons_new(tasksToS, t_var, &pairs, maxDist,filename, t_con, dist_a);
  }
  else
  {
    err = solver->setFinalCons_preVar(tasksToS, t_var, &pairs, maxDist,dist_a);
  }
  if (err != SCIP_OKAY)
    return -1; 


  //conversion from vector to "array"
  SCIP_VAR * array_tvar[numTasks];
  for(int i=0; i<numTasks; i++)
    array_tvar[i] = t_var->at(i);

  bool * worked = new bool();

  //calling solver for the formulated MIP problem
  err = solver->scipSolve(tasksToS, array_tvar, worked, filename, timeout);
   if (err != SCIP_OKAY)
    return -1; 

  //call destructor for SCIP problem
  delete solver;

  return *worked;
}



