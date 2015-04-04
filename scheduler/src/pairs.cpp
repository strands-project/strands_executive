/**
  This class contains methods to determine, which tasks are overlapping, and it establishes the pairs - containing tasks' ids.

  @author Lenka Mudrova
  @version 2.0 04/04/2015
*/
#include "pairs.h"
#include "task.h"
#include <vector>
#include <iostream> 
#include <algorithm>


using namespace std;

/**
  A constructor - initialise the set of tasks
  @param a set of tasks to use
  @return - nan, it is a constructor 
*/
Pairs::Pairs(vector<Task *> * input_tasks)
{
  tasksToS = input_tasks;
  numTasks = tasksToS -> size();
  numPairs = 0;
}

/**
  A destructor
*/
Pairs::~Pairs()
{
  tasksToS = NULL;
  numTasks = 0;
  numPairs = 0;
  pairs.clear();
}

/**
  return determined pairs
  @param a pointer to the vector with pairs
  @return nothing
*/
void Pairs::getPairs(vector<vector<int>> * rp)
{
  *rp = pairs;
}

/**
  This method determines pairs based on the Brian Coltin's article - Dynamic User Task Scheduling for Mobile Robots (2011)
  @param none
  @return number of determined pairs
*/
int Pairs::setPairs_BC()
{
  vector<vector<int>>::iterator it;
  vector<int> opairs(4);  //one pair, always containing two integers (tasks' ids) + an order of a pair, when setted by the preVar method + the type of the pair

  //setting pairs based on the fact if their windows are overlapping. Thus, we need to decide if i precede j, or j precede i
  for (int i=0; i<numTasks; i++)
  {
    for (int j=i+1; j<numTasks; j++)
    {

      bool set = false;
      double ei = tasksToS->at(i)->getEnd();
      double si = tasksToS->at(i)->getStart();
      double ej = tasksToS->at(j)->getEnd();
      double sj = tasksToS->at(j)->getStart();

      double overlap = min(ei,ej) - max(si,sj);

      //I before J, or I after J there is no overlap and we dont want to add the pair.
      //for other cases, we would like to add some constraint
             
      if(overlap > 0)
      {
         opairs[0] = i;
         opairs[1] = j;
         opairs[2] = -1;
         opairs[3] = 2;  //2 - any combination of tasks is possible 
         set = true;     
      }
      
       if(set)
       {
         it = pairs.begin() + numPairs;
         pairs.insert(it,opairs);
         numPairs++;
       }
    }
  }
  return numPairs;
}

/**
  This method check if combination of tasks are even possible and do not create a flaw
  @param overlap represent a time which two tasks have together
  @param consumed is the time consumed by both task + travel
  @param value is expected order
  @return True if tasks are ok, False if there is a flaw
*/
int checkFeasibility(double overlap, double consumed, int value)
{
  if (overlap > consumed)
    return value;
  else
    return -1;
}

/**
  For intervals, where both options (I precedes J, J precedes I) are logically possible (equal, i during j, j during i), 
  we would like to test, if time, which is needed for travel, doesnt exclude one option
  @param pointers to task I and J, time neded to travel from I to J, from J to I
  @return the type of a pair
*/
int decidedInterval(double overlap1, double overlap2, double ij, double ji)
{
  int v1 = checkFeasibility(overlap1, ij, 1);
  int v2 = checkFeasibility(overlap2, ji, 0);

  if((v1>=0)&&(v2>=0))//both combinations are possible
    return 2;
  else
  {
    if(v1>=0) //only combination that i precedes j is possbile during overlap
    {
      return 1;
    }
    else if(v2>=0) //only combination that j precedes i
    {
      return 0;
    }
    else
    {
      return -1; //there is a flaw in input data, both combinations are not possible      
    }
  }

}



/**
  If both situations (I precedes J, J precedes I) are still possible, we simly choose one which minimises the time to execute both tasks
  @param time needed to execute I precedes J, time needed to execute J precedes I
  @return the chosen type of the pair
*/

int chooseInt(int time_ij, int time_ji)
{
  if(time_ij <= time_ji) //added equal
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

/**
  This method is mine proposed approach, which sets the pairs according Allen's interval algebra
  @param a pointer to 2d array of computed distances (this save a computation time)
  @return number of determined pairs if everything is OK
          or -1 when there is a flaw in input data (two tasks cannot be scheduled)
          or -2 if I created a flaw in pairs combinations
*/
int Pairs::setPairs_new(double ** dist_a)
{
  vector<vector<int>>::iterator it;
  vector<int> opairs(4);  //one pair, always containing two integers + order of pair, when setted by preVar method + the type of pair

  int track[numTasks][numTasks]; //an array to keep track of pair preceding variables and detect possible flaws
  for(int a=0; a<numTasks;a++)
    for(int b=0; b<numTasks;b++)
      track[a][b] = 2; //inicialization 

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

      double distij = dist_a[a][b];
      double distji = dist_a[b][a];
   
      //when i precedes j
      double ti = si;
      double tj = si+di+distij;
      if(tj<sj)
        tj = sj;
      //criterion
      double time_ij = (ti+di-si)+(tj+dj-sj);

      //when j precedes i
      tj = sj;
      ti = sj+dj+distji;
      if(ti<si)
        ti = si;
      double time_ji = (ti+di-si)+(tj+dj-sj);

      //old version 
      //double time_ij = si+di+distij - sj;
      //double time_ji = sj+dj+distji - si;

      double ij = di+dj+distij;
      double ji = di+dj+distji;

      double overlap1 = ej-si;
      double overlap2 = ei-sj;

      if (time_ij <0)
        time_ij = 0;
      if (time_ji <0)
        time_ji = 0;
           


      /*
      I before J, or I after J there is no overlapp and we dont want to add any constrain
      If task I ends after task J starts, 
      thus they overlap in certain way, 
      and we need to add constrain to ensure, that their execution will not overlap*/
      if(ei>sj) 
      {        
        //the combination of tasks is possible
        opairs[0] = a;//tasksToS->at(i)->getID();
        opairs[1] = b;//tasksToS->at(j)->getID();
        opairs[2] = -1; // this will be set in preVar method

      
        //i overlaps j
        if((si<sj)&&(ei<ej))
        {
          opairs[3] = checkFeasibility(overlap1,ij,1);          
          set = true;
        }
        
        //i starts j, equal
        else if(si==sj)
        {
          if(ei<ej) //i starts j
          {
            opairs[3] = checkFeasibility(overlap1,ij,1); 
            set = true;
          }
          
          else if (ei==ej)//equals
          {
            /*
            the pair should be of type 2 (any combination), but if intervals are same and we are not using any priority, 
            it makes no sense to chose ordering, 
            however, dist(i,j) or dist(j,i) can be different and they might change the ordering. 
            Thus, we decide here which is the best ordering */

            opairs[3]=decidedInterval(overlap1, overlap2, ij, ji);   
            set = true;
          }
        }
       //j during i
        else if((si<sj)&&(ei>ej))
        {
           //both options (i precedes j, j precedes i) are generally possible
           opairs[3]=decidedInterval(overlap1, overlap2, ij, ji); 
           set = true;
        }
        //i finish j
        else if((ei == ej)&&(si<sj))
        {
          opairs[3]=checkFeasibility(overlap1,ij,1); 
          set = true;
        }
      }

      if(ej>si)
      {
        //the combination of tasks is possible
        opairs[0] = a;//tasksToS->at(i)->getID();
        opairs[1] = b;//tasksToS->at(j)->getID();
        opairs[2] = -1; // this will be set in preVar method

        //j overlaps i
        if((sj<si)&&(ej<ei))
        {
          opairs[3] = checkFeasibility(overlap2,ji,0); 
          set = true;
        }
        
        else if((si==sj)&&(ej < ei)) //j starts i
        {
          opairs[3]= checkFeasibility(overlap2,ji,0); 
          set = true;
        }
       //i during j
        else if((sj<si)&&(ej>ei))
        {
          opairs[3]= decidedInterval(overlap1, overlap2, ij, ji); 
          set = true;
        }
        else if((ei == ej)&&(sj<si))
        {
          opairs[3]= checkFeasibility(overlap2,ji,0);
          set = true;
        }
      }
       if(set)
       {
         if(opairs[3]==-1) //there is a flaw in input data
         {
           return -1;
         }
         else 
         {
           if(track[a][b] == 2) //any combination
           {
             if(opairs[3]==2) //equal or during and we havent got yet constraint
             {
               opairs[3] = chooseInt(time_ij, time_ji); //chose one
             }
             track[a][b] = opairs[3];
             it = pairs.begin() + numPairs;
             pairs.insert(it,opairs);
             numPairs++;
           }
           else //we have allready constraint
           {
             if(opairs[3] == 2) //order doesnt matter, choose the set constraint
             {
               opairs[3] = track[a][b];
             }
             if(track[a][b] == opairs[3]) //constraint are same
             {
               it = pairs.begin() + numPairs;
               pairs.insert(it,opairs);
               numPairs++;
             }
             else //flaw in my pair construction
             {
               return -2;
             }
           } 
         }
       }

    }//end of b loop
    //we have a row of track array, from that, we can set up how the next row should look like
    int act_f = track[a][a+1];
    for(int b=a+2;b<numTasks;b++)
    {
      if(act_f == track[a][b])
      {
        track[a+1][b] = 2; // any combination
      }
      else
      {
        track[a+1][b] = track[a][b];
      }
    }
  }

  return numPairs;
}
