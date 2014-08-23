#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include "task.h"
#include "scheduler.h"
#include "distWrapper.h"
#include <fstream>

using namespace std;

//set these parameters
double ratio[][4] = {{0.01, 0.25},{0.25, 0.5}, {0.5,0.75}, {0.75,1}}; //ratio of task duration/time window
int kind_over[] = {0,1,2}; // only j precedes k fits to overlap, only k precedes j fits to overlap, both are possible


int const mind = 2; //minimal duration of tasks
int const maxd = 30; //maximal duration of tasks
int const maxw = 20; //maximum waiting time between two tasks
int const minsp = 2;
int type = 0;
int over = kind_over[2];
int amount_over = 3; //how many tasks should overlap with existing task 


double act_r[2] = {ratio[0][type], ratio[1][type]}; //actual used ratio
int r=0,s=0,d=0,p=0;
vector<Task*> tasks;
vector<vector<string>> locations;
ofstream myfile;

void noOverlap(int num, int initial_time, int order)
{
  int space, time_win;
  double rand_rat;
  

  for(int i=0; i< num; i++)
  {
    string s1 = locations.at(i).at(0);
    string s2 = locations.at(i).at(1);

    space = (int) round(rand() % maxw)+minsp; 
    rand_rat = (rand() % ((int)((act_r[1] - act_r[0])*100))+act_r[0]*100)/100.0;

    r = initial_time; 
    s =r+space + DistWrapper::dist(s1,s2);
    p = (int) round(rand() % (maxd-mind)+mind);
    time_win = (int) round(p / rand_rat);
    d = max(max(r + time_win, s+p), d+1); //to gurantee that new deadline is bigger
    initial_time = d+1; //+1 to create a space between tasks

    if(order == 1)
    {
      vector<Task*>::iterator iter = tasks.begin() + i;
      tasks.insert(iter,new Task(i, r, d, p, s1, s2));
    }
    else
    {
      vector<Task*>::iterator iter = tasks.end() - i;
      tasks.insert(iter,new Task(i, r, d, p, s1, s2));
    }
  }
}

void overlap(int num, int initial_time, int order)
{
    int aover=0; //counting how many tasks overlap
    int space, time_win;
    double rand_rat;
    int min_overlap = 0;
    int max_overlap = 0;
  
    vector<Task*> previous;
    previous.resize(amount_over-1,(Task*)NULL);

    Task * t = (Task*)NULL;
  
    for(int i=0;i<num;i++)
    {	
      string s1 = locations.at(i).at(0);
      string s2 = locations.at(i).at(1);
      if(aover < amount_over) //we would like to create overlapping tasks
      {
        space = (int) round(rand() % maxw)+minsp;     
        rand_rat = (rand() % ((int)((act_r[1] - act_r[0])*100))+act_r[0]*100)/100.0;

        if(aover == 0) //generation of first task
        {
          //setting of execution time
          r = initial_time; 
          s =r+space + DistWrapper::dist(s1,s2);
          p = (int) round(rand() % (maxd-mind)+mind);
          time_win = (int) round(p / rand_rat);             
        }
        else
        {    
          if(over == 2) //both tasks needs to fit to overlap
          {
            s =s+p+space + DistWrapper::dist(s1,s2);
            p = (int) round(rand() % (maxd-mind)+mind); 
            
            //for first overlap
            t =previous.at(0); 
            max_overlap = t->getEnd() - (t->getDuration() +p+DistWrapper::dist(s1,s2));
            min_overlap = t->getStart()+1; //+1 to ensure that it will start always after first task
            s1 = t->getEndPos();

            for(int j=1; j<aover; j++)
            {
              t =previous.at(j); 
              int maxv = t->getEnd() - (t->getDuration() +p+DistWrapper::dist(s1,s2));
              if (maxv < max_overlap)
                max_overlap = maxv;
            
              int minv = t->getStart()+1; //+1 to ensure that it will start always after first task
              if (minv > min_overlap)
                min_overlap = minv;

              s1 = t->getEndPos();
            }

            r = (int) round(rand() % (max_overlap - min_overlap) + min_overlap);      
            time_win = (int) round(p / rand_rat);
                           
          }        
        }
        d = max(max(r + time_win, s+p), d+1); //to gurantee that new deadline is bigger

        vector<Task*>::iterator iter = previous.begin() + aover;
        previous.insert(iter,new Task(i, r, d, p, s1, s2));
        aover++;

      }
      if(aover>=amount_over) //we would like to set the counter to zero and create task that do not overlap previous ones
      {
        aover = 0;
        initial_time = d+1; //+1 to create a space between tasks
        previous.clear();
      }
      
      if(order == 1)
      {
        vector<Task*>::iterator iter = tasks.begin() + i;
        tasks.insert(iter,new Task(i, r, d, p, s1, s2));
      }
      else
      {
        vector<Task*>::iterator iter = tasks.end() - i;
        tasks.insert(iter,new Task(i, r, d, p, s1, s2));
      }
    }
    t = (Task*)NULL;
    delete t;
}

void starts(int num, int initial_time, int order)
{
    int aover=0; //counting how many tasks overlap
    int space, time_win;
    double rand_rat;
    int min_overlap = 0;
    int max_overlap = 0;

    int pom;
    int first_d =0;
    int max_d =0;
  
    vector<Task*> previous;
    previous.resize(amount_over-1,(Task*)NULL);

    Task * t = (Task*)NULL;
  
    for(int i=0;i<num;i++)
    {	
      string s1 = locations.at(i).at(0);
      string s2 = locations.at(i).at(1);
      if(aover < amount_over) //we would like to create overlapping tasks
      {
        space = (int) round(rand() % maxw)+minsp;     
        rand_rat = (rand() % ((int)((act_r[1] - act_r[0])*100))+act_r[0]*100)/100.0;

        if(aover == 0) //generation of first task
        {
          //setting of execution time
          r = initial_time; 
          s =r+space + DistWrapper::dist(s1,s2);
          p = (int) round(rand() % (maxd-mind)+mind);
          time_win = (int) round(p / rand_rat);     
          pom = i;        
        }
        else
        {    
          if(over == 2) //both tasks needs to fit to overlap
          {
            s =s+p+space + DistWrapper::dist(s1,s2);
            p = (int) round(rand() % (maxd-mind)+mind); 
            
            //for first overlap
            t =previous.at(0); 
            max_overlap = t->getDuration() +p+DistWrapper::dist(s1,s2);
            s1 = t->getEndPos();

            for(int j=1; j<aover; j++)
            {
              t =previous.at(j); 
              int maxv = t->getDuration() +p+DistWrapper::dist(s1,s2);
              if (maxv > max_overlap)
                max_overlap = maxv;

              s1 = t->getEndPos();
            }

            r = initial_time;      
            time_win = (int) round(p / rand_rat);
            t=tasks.at(pom); //we might need to edit first task in overlapping group
            if(t->getStart()+max_overlap > t->getEnd()) //we need to ensure that both tasks fits to the overlap
            { 
              first_d = t->getStart()+max_overlap;
              t->setEnd(first_d);
            }
                           
          }        
        }
        int expec_d = max(max(r + time_win, s+p), first_d+1); //to gurantee that new deadline is bigger
        if(d == expec_d)
          d = expec_d+1;
        else
          d = expec_d;


        if(d>max_d)
          max_d = d;

        if(pom == i)
        {
          first_d = d;
          max_d = d;
        }

        vector<Task*>::iterator iter = previous.begin() + aover;
        previous.insert(iter,new Task(i, r, d, p, s1, s2));
        aover++;

      }
      if(aover>=amount_over) //we would like to set the counter to zero and create task that do not overlap previous ones
      {
        aover = 0;
        initial_time = max_d+1; //+1 to create a space between tasks
        previous.clear();
      }
    
      if(order == 1)
      {
        vector<Task*>::iterator iter = tasks.begin() + i;
        tasks.insert(iter,new Task(i, r, d, p, s1, s2));
      }
      else
      {
        vector<Task*>::iterator iter = tasks.end() - i;
        tasks.insert(iter,new Task(i, r, d, p, s1, s2));
      }
    }
    t =(Task*)NULL;
    delete t;
}

void writeTofile(int num, string name_file)
{
  Task * t =(Task*)NULL;
  myfile.open ("/home/lenka/phd/code/strands_ws/src/strands_executive/scheduler/data/"+name_file + ".txt", std::ios_base::app);
  for(int i=0; i< num; i++)
  {
    t = tasks.at(i);
    cout << t->getStart() << ";" << t->getEnd() << ";" << t->getDuration() << "\n";
  }
  t = (Task*)NULL;
  delete t;
  tasks.clear();
  myfile.close();
}

int main (int argc, char** argv)
{
 
  int initial_time = 0; //if generating more sets, which should be combine 
  
  string name_file = "pokus";

  unsigned int task_count;

  if(argc > 2) {
    task_count = stoi(argv[1]);
    name_file = argv[2];
  }
  else if(argc>1)
  {
    task_count = stoi(argv[1]);
  }
  else {
    task_count = 10;
  }
  
  task_count = task_count*amount_over;
  vector<string> fake_loc;
  fake_loc.resize(2,"office");
  locations.resize(task_count,fake_loc);
  locations.at(0).at(0) = "";
  locations.at(task_count-1).at(1) = "";

  srand(time(NULL));
  /*noOverlap(task_count, initial_time, 1);
  initial_time = tasks.at(task_count-1) -> getEnd()+1;
  writeTofile(task_count, name_file);
  

  overlap(task_count, initial_time, -1);
  initial_time = tasks.at(task_count-1) -> getEnd()+1;
  writeTofile(task_count, name_file);*/

  starts(task_count, initial_time, 1);
  initial_time = tasks.at(task_count-1) -> getEnd()+1;
  writeTofile(task_count, name_file);
  return 0;

}


