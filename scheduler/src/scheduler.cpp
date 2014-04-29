#include <vector>
#include "scheduler.h"
#include "task.h"
#include "distWrapper.h"
#include <iostream> //TODO: delete
#include "scipUser.h"
#include <math.h>  
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
  setPairs();
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

void Scheduler::setPairs()
{
  vector<vector<int>>::iterator it;
  vector<int> opairs(4);  //one pair, always containing two integers + order of pair, when setted by preVar method + the type of pair

  //setting pairs based on the fact if their windows are overlapping. Thus we need to decide if i precede j, or j precede i
  for (int i=0; i<numTasks; i++)
  {
    for (int j=i+1; j<numTasks; j++)
    {
      double ei = tasksToS->at(i)->getEnd();
      double si = tasksToS->at(i)->getStart();
      double ej = tasksToS->at(j)->getEnd();
      double sj = tasksToS->at(j)->getStart();
      //double dist = DistWrapper::dist(tasksToS->at(i)->getEndPos(),tasksToS->at(j)->getStartPos());

      //I before J, or I after J there is no overlapp and we dont want to add the pair.
      //for other cases, we would like to add some constraint
      if(ei>sj)
      {
        //the combination of tasks is possible
        opairs[0] = i;//tasksToS->at(i)->getID();
        opairs[1] = j;//tasksToS->at(j)->getID();
        opairs[2] = -1; // this will be set in preVar method
        opairs[3] = -1;

        //i overlaps j
        if((si<sj)&&(ei<ej))
        {
          opairs[3] = 1;
        }
        
        //i starts j, equal
        else if(si==sj)
        {
          if(ei<ej) //i starts j
          {
            opairs[3]=1;
          }
          
          else if (ei==ej)//equals
          {
            opairs[3]=1; //this should be 2, but if intervals are same and we are not using any priority, it makes no sense
                         //to chose ordering, thus fixing it for task i precedes task j
          }
        }
       //j during i
        else if((si<sj)&&(ei>ej))
        {
           opairs[3]=2;
        }
        //i finish j
        else if((ei == ej)&&(si<sj))
        {
          opairs[3]=1;
        }
        it = pairs.begin() + numPairs;
        pairs.insert(it,opairs);
        numPairs++;
        //cout << opairs[0] << ";" << opairs[1] << opairs[3] << "\n";
      }
      if(ej>si)
      {
        //j overlaps i
        if((sj<si)&&(ej<ei))
        {
          opairs[3] = 0;
        }
        
        else if((si==sj)&&(ej < ei)) //i imeets j
        {
          opairs[3]=0;
        }
       //i during j
        else if((sj<si)&&(ej>ei))
        {
          opairs[3]=2;
        }
        else if((ei == ej)&&(sj<si))
        {
          opairs[3]=0;
        }
        it = pairs.begin() + numPairs;
        pairs.insert(it,opairs);
        numPairs++;
        //cout << opairs[0] << ";" << opairs[1] << opairs[3] << "\n";
      }
      

     // if(ei+dist >sj)
      //{     
      //}
    }
  }
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
  SCIP_Retcode err;
  int i,j;
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
            err = solver->preVar(tid, kid, 1.0, 1.0,order); //seting low and up to same value will fix it to that
            pairs.at(j).at(2) = *order;
            if (err != SCIP_OKAY)
              return -1;
          }
          //if the task is in pair on the second place, it means that variable pre_ij must be zero, because then pre_ji is one
          else if((tasksToS->at(p.at(1))->getID() == tid)&&(tasksToS->at(p.at(0))->getID() == kid))
          { 
            pairSet[j] = true;
            nowSet[k] = true;
            err = solver->preVar(kid, tid, 0.0, 0.0,order); //seting low and up to same value will fix it to that
            pairs.at(j).at(2) = *order;
            if (err != SCIP_OKAY)
              return -1;
          }
        }
        if(!nowSet[k]) //if pair doesnt exist in pairs, we need to set it
        {
           nowSet[k] = true;
           vector<vector<int>>::iterator it;
           it = pairs.begin()+numPairs;
           vector<int> opair(3);
           //to have pair always with smaller number first
           if(i < k)
           {
             err = solver->preVar(tid, kid, 1.0, 1.0,order); //seting low and up to same value will fix it to that
             opair[0] = i;//tid;
             opair[1] = k;//kid;
             opair[2] = *order;
           }
           else
           {
             err = solver->preVar(kid, tid, 0.0, 0.0,order); 
             opair[0] = k;//kid;
             opair[1] = i;//tid;
             opair[2] = *order;
           }

           if (err != SCIP_OKAY)
              return -1;
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
              err = solver->preVar(idt, idp, 0.0, 0.0,order); //seting low and up to same value will fix it to that
              pairs.at(j).at(2) = *order;
              if (err != SCIP_OKAY)
                return -1;
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
              err = solver->preVar(idp, idt, 1.0, 1.0,order); //seting low and up to same value will fix it to that
              pairs.at(j).at(2) = *order;
              if (err != SCIP_OKAY)
                return -1;
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
           vector<int> opair(3);

           if(taskWithCond.at(i) < k)
           {
             err = solver->preVar(idt, idp, 0.0, 0.0,order); //seting low and up to same value will fix it to that
             opair[0] = taskWithCond.at(i);//idp;
             opair[1] = k;//idt;
             opair[2] = *order;
           }
           else
           {
             err = solver->preVar(idp, idt, 1.0, 1.0,order); //seting low and up to same value will fix it to that
             opair[0] = k;//idt;
             opair[1] = taskWithCond.at(i);//idp;
             opair[2] = *order;
           }

           if (err != SCIP_OKAY)
              return -1;
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
      err = solver->preVar(tasksToS->at(p.at(0))->getID(), tasksToS->at(p.at(1))->getID(), 0.0, 1.0,order); 
      pairs.at(j).at(2) = *order;
      if (err != SCIP_OKAY)
        return -1;
     }
   }
    
  
  return 0;
}

bool Scheduler::solve()
{
  SCIP_Retcode err;
  vector<bool> pairUsed;

  ScipUser * solver = new ScipUser();
  err = solver->getEr();
  if (err != SCIP_OKAY)
    return -1;
 //postaru
 err = solver->fakeVar();
  if (err != SCIP_OKAY)
    return -1;

  SCIP_VAR * g = solver->getF();


//creating a vector for variables
  vector<SCIP_VAR *> * t_var = new vector<SCIP_VAR *>(numTasks,(SCIP_VAR*) NULL); 
  err = solver->tVar(numTasks,t_var);
  if (err != SCIP_OKAY)
    return -1;


  int e = setPreVar(solver);
  if (e==-1)
    return -1;

//create constraints for starting and ending time
  err = solver->setTcons(tasksToS, t_var, g);
  if (err != SCIP_OKAY)
    return -1; 

//for all pairs we need to set condition
  err = solver->setFinalCons_long(tasksToS, t_var, g, &pairs);
  if (err != SCIP_OKAY)
    return -1; 

//conversion from vector to "array"
  SCIP_VAR * array_tvar[numTasks];
  for(int i=0; i<numTasks; i++)
    array_tvar[i] = t_var->at(i);

  bool * worked = new bool();

//----------------------- 

  err = solver->scipSolve(tasksToS, array_tvar, worked);
   if (err != SCIP_OKAY)
    return -1; 

  //call destructor
  delete solver;

  return *worked;
}



