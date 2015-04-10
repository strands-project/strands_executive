/**
  This class contains usage of SCIP solver. 

  @author Lenka Mudrova
  @version 1.0 29/10/2014
*/

#include "scipUser.h"
#include "task.h"


/* scip includes */
#include "objscip/objscip.h"
#include "objscip/objscipdefplugins.h"

#include <vector>

#include <chrono>

#include <iostream> //TODO:delete this
#include <fstream>

using namespace scip;
using namespace std;

/**
  The constructor - initialise the SCIP solver
  @param none
  @return nan, it is constructor
*/
ScipUser::ScipUser()
{
  scip = new SCIP();//NULL;
  /* initialize SCIP environment */
  catchEr = SCIPcreate(&scip);
  /* include default plugins */  //TODO: figure out what exatly this is doing
  catchEr = SCIPincludeDefaultPlugins(scip);
  /* set verbosity parameter */
  //TODO:not sure about this
  catchEr = SCIPsetIntParam(scip, "display/verblevel", 5);
  /* create empty problem */
  catchEr = SCIPcreateProb(scip, "Scheduler", 0, 0, 0, 0, 0, 0, 0);
}

/**
  The destructor - release scip from a memory
*/
ScipUser::~ScipUser()
{
  catchEr = SCIPfree(&scip);
  BMScheckEmptyMemory();
}

/**
  return the scip error
  @param none
  @return the scip error
*/
SCIP_Retcode ScipUser::getEr()
{
  return catchEr;
}

/**
  Creates t-variables (execution time of a task) for all tasks
  @param number of tasks, a vector of pointers to the variables
  @return scip_retcode
*/
SCIP_Retcode ScipUser::tVar(int num_tasks, vector<SCIP_VAR *> * t_var)
{
  /* add t variables (execution of task) */
 
   char var_name[255];
   vector<SCIP_VAR*>::iterator it;
   for (int i = 0; i < num_tasks; i++)
   {
      SCIP_VAR* var;
      SCIPsnprintf(var_name, 255, "t_%d", i);

      SCIP_CALL( SCIPcreateVar(scip,
                     &var,                   // returns new index
                     var_name,               // name
                     0.0,                    // lower bound
                     SCIP_DEFAULT_INFINITY,                    // upper bound
                     1,         // objective reflects priority, it multiplies time of the variable to affect global optimum
                     SCIP_VARTYPE_CONTINUOUS,   // variable type
                     true,                   // initial
                     false,                  // forget the rest ...
                     0, 0, 0, 0, 0) );
      SCIP_CALL( SCIPaddVar(scip, var) );
      it = t_var->begin()+i;
      t_var->insert(it,var);  
   }
   return SCIP_OKAY;
}

/**
  creates a single pre-variable used for constraints, where two tasks can overlap
  @param vector of pointers to variables, two indexes of tasks, which are overlapping
  @return scip_retcode
*/
SCIP_Retcode ScipUser::preVar(vector<SCIP_VAR *> * pre_var, int i, int j)
{
  /* add pre variables (execution of task) */
 
   char var_name[255];
   SCIP_VAR * var;
   SCIPsnprintf(var_name, 255, "pre_%d%d", i,j);

   SCIP_CALL( SCIPcreateVar(scip,
                     &var,                   // returns new index
                     var_name,               // name
                     0.0,                    // lower bound
                     1.0,                    // upper bound
                     0.0,         // objective reflects priority, it multiplies time of the variable to affect global optimum
                     SCIP_VARTYPE_BINARY,   // variable type
                     true,                   // initial
                     false,                  // forget the rest ...
                     0, 0, 0, 0, 0) );
   SCIP_CALL( SCIPaddVar(scip, var) ); 
   pre_var->insert(pre_var->begin(),var);
   
   return SCIP_OKAY;
}

/**
  set a constraint s <= t <= e for a single task
  @param index to the t-variable, vector of pointers to t-variables , vector of pointers to the t- constrains, start limit, end limit
  @return scip_retcode
*/
SCIP_Retcode ScipUser::setOneTcons(int i, vector<SCIP_VAR *> * t_var, vector<SCIP_CONS*> * t_con, int s, int x)
{
  char con_name[255];
  SCIP_VAR* ti = (SCIP_VAR*)NULL;
  ti = t_var->at(i);

  SCIP_CONS* con = (SCIP_CONS*)NULL;
  SCIPsnprintf(con_name, 255, "s_%d", i);

  SCIP_VAR * vars0[1];
  vars0[0] = ti;

  SCIP_Real vals0[1];
  vals0[0] = 1.0;

  SCIP_CALL(SCIPcreateConsLinear (scip,
		&con,
		con_name,
		1, //number of variables
		vars0,//&vars,
		vals0,
		s,//  	lhs,
		x,//SCIP_DEFAULT_INFINITY,//  	rhs,
		true,   // 	initial,
		true,    //  	separate,
		true,  //  	enforce,
		true,  //  	check,
		true,  //  	propagate,
		false, // 	local,
		false, //  	modifiable,
		false, //  	dynamic,
		false,//  	removable,
		false//  	stickingatnode
  ));
  

  /*SCIP_CONS* con2 = (SCIP_CONS*)NULL;
  SCIPsnprintf(con_name, 255, "e_%d", i);

  SCIP_CALL(SCIPcreateConsLinear (scip,
		&con2,
		con_name,
		1, //number of variables
		vars0,//&vars,
		vals0,
		-SCIP_DEFAULT_INFINITY,//  	lhs,
		x,//  	rhs,
		true,   // 	initial,
		true,    //  	separate,
		true,  //  	enforce,
		true,  //  	check,
		true,  //  	propagate,
		false, // 	local,
		false, //  	modifiable,
		false, //  	dynamic,
		false,//  	removable,
		false//  	stickingatnode
  ));*/

  //create a conjunction
  /*SCIP_CONS* conj;
  SCIPsnprintf(con_name, 255, "junse_%d", i);
  SCIP_CONS* arr_jun[2];
  arr_jun[0] = con;
  arr_jun[1] = con2;
  SCIP_CALL(SCIPcreateConsConjunction( scip,
                &conj,
		con_name,
                2,
                arr_jun,
                true,
                true,
                false,
                false,
                false
  ));*/

   //old
  //SCIP_CALL( SCIPaddCons(scip, conj) );
  SCIP_CALL( SCIPaddCons(scip, con) );
  //SCIP_CALL( SCIPreleaseCons(scip, &con));
  //SCIP_CALL( SCIPreleaseCons(scip, &con2));

  //SCIP_CALL(SCIPprintCons(scip,	conj, NULL));
  //cout << "\n"; 	
  //old
  //t_con->at(i) = conj;	

  t_con->at(i) = con;
  
  //delete con;
  //con = NULL;
  //delete con2;
  //con2 = NULL;


  
  return SCIP_OKAY;
}

/**
  set constrain s<= t <= e to all tasks
  @param vector of pointers to tasks, vector of pointers to t-variables, vector of pointer to t-constraints
  @return scip_retcode
*/
SCIP_Retcode ScipUser::setTcons(vector<Task*> * tasksToS, vector<SCIP_VAR *> * t_var, vector<SCIP_CONS*> * t_con)
{
  /* add constraint s<= t + d <=e */
 
  for (int i = 0; i < tasksToS->size(); i++)
  {    
    SCIP_Real d = tasksToS->at(i)->getDuration(); 
    SCIP_Real s = tasksToS->at(i)->getStart();
    SCIP_Real e = tasksToS->at(i)->getEnd();

    SCIP_CALL(setOneTcons(i, t_var, t_con, s, e-d));   
  }
  return SCIP_OKAY;
}


/**
  This method check the type of a pair and set the final constrain based on it (to ensure, that no tasks overlap)
  @param tasks to schedule, t-variables, pairs, maximal distance between two locations in the scheduling problem, filename to save time, constrains, array of distances
  @return scip_retcode 
*/
SCIP_Retcode ScipUser::setFinalCons_new(vector<Task*> * tasksToS, vector<SCIP_VAR *> * t_var, vector<vector<int>> * pairs, double maxDist, string filename, vector<SCIP_CONS*> * t_con, double ** dist_a)
{
  char con_name[255]; 
  ofstream results;
  int numinfeas = 0;

  for(int x=0; x<(int)pairs->size(); x++)
  {
     vector<int> p = pairs->at(x);
     int i = p.at(0);
     int j = p.at(1);
     //int k = p.at(2);
     int type = p.at(3);

     SCIP_VAR* ti;
     SCIP_VAR* tj;
   
     ti = t_var->at(i);
     tj = t_var->at(j);

     SCIP_CONS* con;
     SCIP_CONS* con2;
    
     SCIP_Real distij, distji;
    
     if(type==1) //task i should precede j,
     {   
       //creating a constraint ti + di + dist - tj <= 0     
       SCIP_Real di = tasksToS->at(i)->getDuration();

       if(tasksToS->at(i)->getEndPos().empty()) //if first task has no location, that the travel to following task might take maxDist;
       {
         distij = maxDist;
       }
       else if(tasksToS->at(j)->getStartPos().empty()) //if second task in pair has no location, travel dist is zero, should start immediately
       {
         distij = 0; 
       }
       else
       {        
         distij = dist_a[i][j];
       }
        //two following tasks with no loc
       if((tasksToS->at(i)->getEndPos().empty())&&(tasksToS->at(j)->getStartPos().empty()))
       { 
         distij =0;
       }
       SCIP_Real vals[2]; //array of values
       vals[0] = 1;
       vals[1] = -1; 

       double rhs = -di-distij; 
     
       SCIP_VAR * vars[2];
       vars[0] = ti;
       vars[1] = tj; 
 
       
       SCIPsnprintf(con_name, 255, "tddt_%d%d", i,j);

       SCIP_CALL(SCIPcreateConsLinear 	(scip,
		&con,
		con_name,
		2, //number of variables
		vars,//&vars,
		vals,
		-SCIP_DEFAULT_INFINITY,//  	lhs,
		rhs,//  	rhs,
		true,   // 	initial,
		true,    //  	separate,
		true,  //  	enforce,
		true,  //  	check,
		true,  //  	propagate,
		false, // 	local,
		false, //  	modifiable,
		false, //  	dynamic,
		false,//  	removable,
		false//  	stickingatnode
	) );
       SCIP_CALL( SCIPaddCons(scip, con));
       SCIP_CALL( SCIPreleaseCons(scip, &con));
     }
     if(type==0) //task j precedes task i
     {
       //creating a constraint tj + dj + dist - ti <= 0
       

       SCIP_Real dj = tasksToS->at(j)->getDuration();

       if(tasksToS->at(j)->getEndPos().empty()) //if first task has no location, that the travel to following task might take maxDist;
       {
         distji = maxDist;
       }
       else if(tasksToS->at(i)->getStartPos().empty()) //if second task in pair has no location, travel dist is zero, should start immediately
       {
         distji = 0; 
       }
       else
       {  
      
         distji = dist_a[j][i];
       }
       if((tasksToS->at(j)->getEndPos().empty())&&(tasksToS->at(i)->getStartPos().empty()))
       {
         distji = 0;
       }

       SCIP_Real vals2[2]; //array of values
       vals2[0] = 1;
       vals2[1] = -1;  
     
       double rhs2 = -dj-distji;
       SCIP_VAR * vars2[2];
       vars2[0] = tj;
       vars2[1] = ti; 

 
       SCIPsnprintf(con_name, 255, "tddt_%d%d",j,i);

       SCIP_CALL(SCIPcreateConsLinear 	(scip,
		&con2,
		con_name,
		2, //number of variables
		vars2,//&vars,
		vals2,
		-SCIP_DEFAULT_INFINITY,//  	lhs,
		rhs2,//  	rhs,
		true,   // 	initial,
		true,    //  	separate,
		true,  //  	enforce,
		true,  //  	check,
		true,  //  	propagate,
		false, // 	local,
		false, //  	modifiable,
		false, //  	dynamic,
		false,//  	removable,
		false//  	stickingatnode
	) );
       SCIP_CALL( SCIPaddCons(scip, con2));
       SCIP_CALL( SCIPreleaseCons(scip, &con2));
    }

    if(type==2)
    {
      SCIP_CALL( setFullConstr(tasksToS, t_var, i, j, maxDist,dist_a));
    }
  }

  if(!filename.empty())
  {
     results.open (filename,std::ios_base::app);
     results << numinfeas << " ";
  }
  return SCIP_OKAY;
}


/**
  This method sets a single constrain, when both options are possible (I precedes J, J precedes I)
  @param tasks to schedule, t-variables, index to task i, index to task j, maximal distance, array of distances
  @return scip retcode
*/
SCIP_Retcode ScipUser::setFullConstr(vector<Task*> * tasksToS, vector<SCIP_VAR *> * t_var, int i, int j, double maxDist, double ** dist_a)
{
   char con_name[255]; 

   SCIP_VAR* ti;
   SCIP_VAR* tj;
   
   ti = t_var->at(i);
   tj = t_var->at(j);

   SCIP_CONS* con;
   SCIP_CONS* con2;
   //create preVar
       vector<SCIP_VAR *> * pre_var = new vector<SCIP_VAR *>(1,(SCIP_VAR*) NULL); 
       preVar(pre_var, i, j);
       SCIP_Real di = tasksToS->at(i)->getDuration();
       SCIP_Real dj = tasksToS->at(j)->getDuration();

       /*-------------------------------------------------*/
       //setting variable distij based on positions
       SCIP_Real distij;
       if(tasksToS->at(i)->getEndPos().empty()) //if first task has no location, that the travel to following task might take maxDist;
       {
         distij = maxDist;
       }
       else if(tasksToS->at(j)->getStartPos().empty()) //if second task in pair has no location, travel dist is zero, should start immediately
       {
         distij = 0; 
       }
       else
       {        
         distij = dist_a[i][j];
       }
        //two following tasks with no loc
       if((tasksToS->at(i)->getEndPos().empty())&&(tasksToS->at(j)->getStartPos().empty()))
       { 
         distij =0;
       }

       /*-------------------------------------------------*/
       //setting variable distij based on positions
       SCIP_Real distji;
       if(tasksToS->at(j)->getEndPos().empty()) //if first task has no location, that the travel to following task might take maxDist;
       {
         distji = maxDist;
       }
       else if(tasksToS->at(i)->getStartPos().empty()) //if second task in pair has no location, travel dist is zero, should start immediately
       {
         distji = 0; 
       }
       else
       {        
         distji = dist_a[j][i];
       }
       if((tasksToS->at(j)->getEndPos().empty())&&(tasksToS->at(i)->getStartPos().empty()))
       {
         distji = 0;
       }

       /*-------------------------------------------------*/
       //creating a constraint ti + di + dist - tj <= (ei+distij-sj)(1-pre)  

       double left_const = tasksToS->at(i)->getEnd() + distij - tasksToS->at(j)->getStart();   
       //original version
       //double rhs = -di-distij+left_const; 
       //mine change
       double rhs = -di-distij;
  
       
       SCIP_Real vals[3]; //array of values
       vals[0] = 1;
       vals[1] = -1; 

       //orignal
       //vals[2] = left_const;
       vals[2] = -left_const;
           
       SCIP_VAR * vars[3];
       vars[0] = ti;
       vars[1] = tj; 
       vars[2] = pre_var ->at(0);

       SCIPsnprintf(con_name, 255, "tddt_%d%d", i,j);

       SCIP_CALL(SCIPcreateConsLinear 	(scip,
		&con,
		con_name,
		3, //number of variables
		vars,//&vars,
		vals,
		-SCIP_DEFAULT_INFINITY,//  	lhs,
		rhs,//  	rhs,
		true,   // 	initial,
		true,    //  	separate,
		true,  //  	enforce,
		true,  //  	check,
		true,  //  	propagate,
		false, // 	local,
		false, //  	modifiable,
		false, //  	dynamic,
		false,//  	removable,
		false//  	stickingatnode
	) );

        /*-------------------------------------------------*/
        //creating a constraint tj + dj + dist - ti <= (ej+distji-si)*pre
        double left_const2 = tasksToS->at(j)->getEnd() + distji - tasksToS->at(i)->getStart();  
        //original
        //double rhs2 = -dj-distji;
        double rhs2 = -dj - distji + left_const2;
        
        SCIP_Real vals2[3]; //array of values
        vals2[0] = 1;
        vals2[1] = -1; 
        // original
        //vals2[2] =  -left_const2;
        vals2[2] = left_const2;
       
        SCIP_VAR * vars2[2];
        vars2[0] = tj;
        vars2[1] = ti;
        vars2[2] = pre_var->at(0);

        SCIPsnprintf(con_name, 255, "tddt_%d%d",j,i);

        SCIP_CALL(SCIPcreateConsLinear 	(scip,
		&con2,
		con_name,
		3, //number of variables
		vars2,//&vars,
		vals2,
		-SCIP_DEFAULT_INFINITY,//  	lhs,
		rhs2,//  	rhs,
		true,   // 	initial,
		true,    //  	separate,
		true,  //  	enforce,
		true,  //  	check,
		true,  //  	propagate,
		false, // 	local,
		false, //  	modifiable,
		false, //  	dynamic,
		false,//  	removable,
		false//  	stickingatnode
	) );

       SCIP_CONS* confinal;
       SCIPsnprintf(con_name, 255, "final_%d%d",i,j);
       SCIP_CONS* arr_final[2];
       arr_final[0] = con;
       arr_final[1] = con2;

       SCIP_CALL(SCIPcreateConsConjunction(scip,
		&confinal,  	//cons,
		con_name, 	//name,
		2,
		arr_final,  	//conss,
		true, //  	enforce,
		true, //  	check,
		false, //  	local,
		false, // 	modifiable,
		false //  	dynamic 
	));

       SCIP_CALL( SCIPaddCons(scip, confinal) ); 
       SCIP_CALL( SCIPreleaseCons(scip, &con));
       SCIP_CALL( SCIPreleaseCons(scip, &con2));
       SCIP_CALL( SCIPreleaseCons(scip, &confinal));	

       delete pre_var;
       pre_var = NULL;

  return SCIP_OKAY;
}

/**
  This is used by original Brian Coltin approach - all constraints (ensuring that tasks do not overlap) are treated as type=2, thus full constrain is added
  @param tasks to schedule, t-variables, pairs, maximal distance, array of distances
  @return scip retcode
*/ 
SCIP_Retcode ScipUser::setFinalCons_preVar(vector<Task*> * tasksToS, vector<SCIP_VAR *> * t_var, vector<vector<int>> * pairs, double maxDist, double ** dist_a)
{
 
  for(int x=0; x<(int)pairs->size(); x++)
  {

     vector<int> p = pairs->at(x);
     int i = p.at(0);
     int j = p.at(1);
     //int k = p.at(2);
     int type = p.at(3);


     if(type == 2) //this method is called for BC original algorithm, all pairs are of type 2
     {
       setFullConstr(tasksToS, t_var, i, j, maxDist, dist_a);
     }
  }
  return SCIP_OKAY;
}

/**
  The main method, where the solving of formulated MIP problem is called
  @param tasks to schedule, variables to find, pointer to boolean (signaling if the solver found solution), filename to save time and criterion, timeout
  @return scip retcode
*/
SCIP_Retcode ScipUser::scipSolve(vector<Task*> * tasksToS, SCIP_VAR * vars[], bool * worked, string filename, const int & timeout)
{
  int num_tasks = tasksToS -> size();
  std::chrono::high_resolution_clock::time_point start, end;
  ofstream results, schedule_file;

  SCIP_Real vals[num_tasks]; //array to save execution times
  start = std::chrono::high_resolution_clock::now();

  //cancel the output to the terminal
  SCIPsetMessagehdlr(scip, NULL);

  if(timeout > 0) {
    SCIP_CALL( SCIPsetRealParam(scip, "limits/time", timeout) );
  }


	
  SCIP_CALL( SCIPsolve(scip) );
  end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_seconds = end-start;
 

  SCIP_CALL( SCIPprintBestSol(scip, NULL, TRUE) );
  SCIP_SOL* sol = SCIPgetBestSol(scip);
  if(!filename.empty())
  {
    results.open (filename,std::ios_base::app);
    results << SCIPgetSolOrigObj(scip,sol) << " " << elapsed_seconds.count() << " " << num_tasks;
  }

  //cout << "criterion:" <<  SCIPgetSolOrigObj(scip,sol);

  if(sol == NULL)
  {
    *worked = false;
    if(!filename.empty())
    {
      results << " " << 0 << "\n";
    }
    for(int i=0; i < num_tasks; i++)
    {
      tasksToS->at(i)->setExecTime(-1.0);
    }
  }
  else
  {
    *worked = true;
    if(!filename.empty())
    {
      results << " " << 1 << "\n";
      schedule_file.open(filename+"schedule.txt",std::ios_base::app);
    }
    SCIP_CALL(SCIPgetSolVals(scip,sol, num_tasks, vars, vals)); 
    
    for(int i=0; i < num_tasks; i++)
    {
      tasksToS->at(i)->setExecTime(vals[i]);
      if(!filename.empty())
        schedule_file << vals[i] << " ";
    }
    if(!filename.empty()){
      schedule_file <<"\n";
      schedule_file.close();
    }
  }	
  results.close();
  return SCIP_OKAY;
}
