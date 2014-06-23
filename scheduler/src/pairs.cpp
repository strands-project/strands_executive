#include "pairs.h"
#include "task.h"
#include <vector>
#include <iostream> 
#include <algorithm>
#include "distWrapper.h"

using namespace std;

Pairs::Pairs(vector<Task *> * input_tasks)
{
  tasksToS = input_tasks;
  numTasks = tasksToS -> size();
  numPairs = 0;
}

void Pairs::getPairs(vector<vector<int>> * rp)
{
  *rp = pairs;
}

int Pairs::setPairs_BC()
{
  vector<vector<int>>::iterator it;
  vector<int> opairs(4);  //one pair, always containing two integers + order of pair, when setted by preVar method + the type of pair

  //setting pairs based on the fact if their windows are overlapping. Thus we need to decide if i precede j, or j precede i
  for (int i=0; i<numTasks; i++)
  {
    for (int j=i+1; j<numTasks; j++)
    {

      bool set = false;
      double ei = tasksToS->at(i)->getEnd();
      double si = tasksToS->at(i)->getStart();
      double ej = tasksToS->at(j)->getEnd();
      double sj = tasksToS->at(j)->getStart();
      //double dist = DistWrapper::dist(tasksToS->at(i)->getEndPos(),tasksToS->at(j)->getStartPos());

      double overlap = min(ei,ej) - max(si,sj);

      //I before J, or I after J there is no overlapp and we dont want to add the pair.
      //for other cases, we would like to add some constraint
       
      //the combination of tasks is possible
      
      if(overlap > 0)
      {
         opairs[0] = i;
         opairs[1] = j;
         opairs[2] = -1;
         opairs[3] = 2;   
         set = true;     
      }
      
       if(set)
       {
         it = pairs.begin() + numPairs;
         pairs.insert(it,opairs);
         numPairs++;
         //cout << opairs[0] << ";" << opairs[1] << opairs[3] << "\n";
       }
    }
  }
  return numPairs;
}

int Pairs::setPairs()
{
  vector<vector<int>>::iterator it;
  vector<int> opairs(4);  //one pair, always containing two integers + order of pair, when setted by preVar method + the type of pair

  //setting pairs based on the fact if their windows are overlapping. Thus we need to decide if i precede j, or j precede i
  for (int i=0; i<numTasks; i++)
  {
    for (int j=i+1; j<numTasks; j++)
    {

      bool set = false;
      double ei = tasksToS->at(i)->getEnd();
      double si = tasksToS->at(i)->getStart();
      double ej = tasksToS->at(j)->getEnd();
      double sj = tasksToS->at(j)->getStart();
      //double dist = DistWrapper::dist(tasksToS->at(i)->getEndPos(),tasksToS->at(j)->getStartPos());

      //I before J, or I after J there is no overlapp and we dont want to add the pair.
      //for other cases, we would like to add some constraint
       
      //the combination of tasks is possible
      opairs[0] = i;//tasksToS->at(i)->getID();
      opairs[1] = j;//tasksToS->at(j)->getID();
      opairs[2] = -1; // this will be set in preVar method
      opairs[3] = -1;

      if(ei>sj)
      {
 
        //i overlaps j
        if((si<sj)&&(ei<ej))
        {
          opairs[3] = 1;
          set = true;
        }
        
        //i starts j, equal
        else if(si==sj)
        {
          if(ei<ej) //i starts j
          {
            opairs[3]=1;
            set = true;
          }
          
          else if (ei==ej)//equals
          {
            opairs[3]=1; //this should be 2, but if intervals are same and we are not using any priority, it makes no sense
                         //to chose ordering, thus fixing it for task i precedes task j
            set = true;
          }
        }
       //j during i
        else if((si<sj)&&(ei>ej))
        {
           opairs[3]=2;
           set = true;
        }
        //i finish j
        else if((ei == ej)&&(si<sj))
        {
          opairs[3]=1;
          set = true;
        }
      }
      if(ej>si)
      {
        //j overlaps i
        if((sj<si)&&(ej<ei))
        {
          opairs[3] = 0;
          set = true;
        }
        
        else if((si==sj)&&(ej < ei)) //i imeets j
        {
          opairs[3]=0;
          set = true;
        }
       //i during j
        else if((sj<si)&&(ej>ei))
        {
          opairs[3]=2;
          set = true;
        }
        else if((ei == ej)&&(sj<si))
        {
          opairs[3]=0;
          set = true;
        }
      }
       if(set)
       {
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
  return numPairs;
}

int decidedInterval(Task * i, Task * j, int logOrder)
{
  double si = i->getStart();
  double ei = i->getEnd();
  double di = i->getDuration();

  double sj = j->getStart();
  double ej = j->getEnd();
  double dj = j->getDuration();

  double distij = DistWrapper::dist(i->getEndPos(),j->getStartPos());
  double distji = DistWrapper::dist(j->getEndPos(),i->getStartPos());

  double ij = di+dj+distij;
  double ji = di+dj+distji;

  double overlap = min(ei,ej) - max(si,sj);

  
  if((ij<=overlap)&&(ji<=overlap)) //both combinations are possible
  {
    return 2;
  }
  else
  {
    if(ij<=overlap) //only combination that i precedes j is possbile during overlap
    {
      if(logOrder==1) //if logic order correspondence to this, return it
      {
        return 1;
      }
      else //logic order should be i precedes j, but there is also option for j precedes i... so theoretically both are possible
      { 
        return 2;
      }
    }
    else if(ji<=overlap) //only combination that j precedes i
    {
      if(logOrder==0) 
      {
        return 0;
      }
      else
      { 
        return 2;
      }
    }
    else
    {
      return logOrder; //the overlapp is too small, none combination can be there, therefore follow logic order
    }
  }


}

int Pairs::setPairs_mine()
{
  vector<vector<int>>::iterator it;
  vector<int> opairs(4);  //one pair, always containing two integers + order of pair, when setted by preVar method + the type of pair

  //setting pairs based on the fact if their windows are overlapping. Thus we need to decide if i precede j, or j precede i
  for (int a=0; a<numTasks; a++)
  {
    for (int b=a+1; b<numTasks; b++)
    {

      bool set = false;
      Task * i = tasksToS->at(a);
      Task * j = tasksToS->at(b);

      double ei = i->getEnd();
      double si = i->getStart();
      double di = i->getDuration();
      double ej = j->getEnd();
      double sj = j->getStart();
      double dj = j->getDuration();

      double distij = DistWrapper::dist(i->getEndPos(),j->getStartPos());
      double distji = DistWrapper::dist(j->getEndPos(),i->getStartPos());

      double time_ij = si+di+distij - sj;
      double time_ji = sj+dj+distji - si;

      if (time_ij <0)
        time_ij = 0;
      if (time_ji <0)
        time_ji = 0;
       
      //the combination of tasks is possible
      opairs[0] = a;//tasksToS->at(i)->getID();
      opairs[1] = b;//tasksToS->at(j)->getID();
      opairs[2] = -1; // this will be set in preVar method
      opairs[3] = -1;


      /*
      I before J, or I after J there is no overlapp and we dont want to add any constrain
      If task I ends after task J starts, 
      thus they overlap in certain way, 
      and we need to add constrain to ensure, that their execution will not overlap*/
      if(ei>sj) 
      {        
        //i overlaps j
        if((si<sj)&&(ei<ej))
        {
          opairs[3] = decidedInterval(i,j,1);
          set = true;
        }
        
        //i starts j, equal
        else if(si==sj)
        {
          if(ei<ej) //i starts j
          {
            opairs[3]=decidedInterval(i,j,1);
            set = true;
          }
          
          else if (ei==ej)//equals
          {
            /*
            this should be 2, but if intervals are same and we are not using any priority, 
            it makes no sense to chose ordering, 
            however, dist(i,j) or dist(j,i) can be different and they might change the ordering. 
            Thus, we decide here which is the best ordering */
            if(time_ij < time_ji)
              opairs[3]=decidedInterval(i,j,2);
            else
              opairs[3]=decidedInterval(i,j,2);
           
            set = true;
          }
        }
       //j during i
        else if((si<sj)&&(ei>ej))
        {
           opairs[3]=decidedInterval(i,j,2);
           set = true;
        }
        //i finish j
        else if((ei == ej)&&(si<sj))
        {
          opairs[3]=decidedInterval(i,j,1);
          set = true;
        }
      }
      if(ej>si)
      {
        //j overlaps i
        if((sj<si)&&(ej<ei))
        {
          opairs[3] = decidedInterval(i,j,0);
          set = true;
        }
        
        else if((si==sj)&&(ej < ei)) //i imeets j
        {
          opairs[3]= decidedInterval(i,j,0);
          set = true;
        }
       //i during j
        else if((sj<si)&&(ej>ei))
        {
          opairs[3]= decidedInterval(i,j,2);
          set = true;
        }
        else if((ei == ej)&&(sj<si))
        {
          opairs[3]= decidedInterval(i,j,0);
          set = true;
        }
      }
       if(set)
       {
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
  return numPairs;
}

int chooseInt(int time_ij, int time_ji)
{
  if(time_ij < time_ji)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

int Pairs::setPairs_new()
{
  vector<vector<int>>::iterator it;
  vector<int> opairs(4);  //one pair, always containing two integers + order of pair, when setted by preVar method + the type of pair

  //setting pairs based on the fact if their windows are overlapping. Thus we need to decide if i precede j, or j precede i
  for (int a=0; a<numTasks; a++)
  {
    for (int b=a+1; b<numTasks; b++)
    {

      bool set = false;
      Task * i = tasksToS->at(a);
      Task * j = tasksToS->at(b);

      double ei = i->getEnd();
      double si = i->getStart();
      double di = i->getDuration();
      double ej = j->getEnd();
      double sj = j->getStart();
      double dj = j->getDuration();

      double distij = DistWrapper::dist(i->getEndPos(),j->getStartPos());
      double distji = DistWrapper::dist(j->getEndPos(),i->getStartPos());

      double time_ij = si+di+distij - sj;
      double time_ji = sj+dj+distji - si;

      if (time_ij <0)
        time_ij = 0;
      if (time_ji <0)
        time_ji = 0;
       
      //the combination of tasks is possible
      opairs[0] = a;//tasksToS->at(i)->getID();
      opairs[1] = b;//tasksToS->at(j)->getID();
      opairs[2] = -1; // this will be set in preVar method
      opairs[3] = -1;


      /*
      I before J, or I after J there is no overlapp and we dont want to add any constrain
      If task I ends after task J starts, 
      thus they overlap in certain way, 
      and we need to add constrain to ensure, that their execution will not overlap*/
      if(ei>sj) 
      {        
        //i overlaps j
        if((si<sj)&&(ei<ej))
        {
          opairs[3] = decidedInterval(i,j,1);
          if(opairs[3] == 2)
            opairs[3] = chooseInt(time_ij, time_ji);
          
          set = true;
        }
        
        //i starts j, equal
        else if(si==sj)
        {
          if(ei<ej) //i starts j
          {
            opairs[3]=decidedInterval(i,j,1);
            if(opairs[3] == 2)
              opairs[3] = chooseInt(time_ij, time_ji);
            set = true;
          }
          
          else if (ei==ej)//equals
          {
            /*
            this should be 2, but if intervals are same and we are not using any priority, 
            it makes no sense to chose ordering, 
            however, dist(i,j) or dist(j,i) can be different and they might change the ordering. 
            Thus, we decide here which is the best ordering */
            opairs[3]=decidedInterval(i,j,2);
            if(opairs[3] == 2)
              opairs[3] = chooseInt(time_ij, time_ji);
        
            set = true;
          }
        }
       //j during i
        else if((si<sj)&&(ei>ej))
        {
           opairs[3]=decidedInterval(i,j,2);
           set = true;
        }
        //i finish j
        else if((ei == ej)&&(si<sj))
        {
          opairs[3]=decidedInterval(i,j,1);
          if(opairs[3] == 2)
            opairs[3] = chooseInt(time_ij, time_ji);
          set = true;
        }
      }
      if(ej>si)
      {
        //j overlaps i
        if((sj<si)&&(ej<ei))
        {
          opairs[3] = decidedInterval(i,j,0);
          if(opairs[3] == 2)
            opairs[3] = chooseInt(time_ij, time_ji);
          set = true;
        }
        
        else if((si==sj)&&(ej < ei)) //i imeets j
        {
          opairs[3]= decidedInterval(i,j,0);
          if(opairs[3] == 2)
            opairs[3] = chooseInt(time_ij, time_ji);
          set = true;
        }
       //i during j
        else if((sj<si)&&(ej>ei))
        {
          opairs[3]= decidedInterval(i,j,2);
          set = true;
        }
        else if((ei == ej)&&(sj<si))
        {
          opairs[3]= decidedInterval(i,j,0);
          if(opairs[3] == 2)
            opairs[3] = chooseInt(time_ij, time_ji);
          set = true;
        }
      }
       if(set)
       {
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
  return numPairs;
}
