#include "scipUser.h"
#include "task.h"
#include "distWrapper.h"

/* scip includes */
#include "objscip/objscip.h"
#include "objscip/objscipdefplugins.h"

#include <vector>

#include <chrono>

#include <iostream> //TODO:delete this
#include <fstream>

using namespace scip;
using namespace std;

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

ScipUser::~ScipUser()
{
  catchEr = SCIPfree(&scip);
  BMScheckEmptyMemory();
}

SCIP_Retcode ScipUser::getEr()
{
  return catchEr;
}

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
                     1.0,         // objective reflects priority, it multiplies time of the variable to affect global optimum
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
		SCIP_DEFAULT_INFINITY,//  	rhs,
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
  

  SCIP_CONS* con2 = (SCIP_CONS*)NULL;
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
  ));

  //create a conjunction
  SCIP_CONS* conj;
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
  ));

  SCIP_CALL( SCIPaddCons(scip, conj) );
  t_con->at(i) = conj;	

  /*
  delete ti;
  ti = NULL;
  delete con;
  con = NULL;
  delete con2;
  con2 = NULL;
  delete vars0;
  vars = NULL;
  delete conj;
  conj = NULL;*/
  
  return SCIP_OKAY;
}

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


SCIP_Retcode ScipUser::setFinalCons(vector<Task*> * tasksToS, vector<SCIP_VAR *> * t_var, vector<vector<int>> * pairs, double maxDist)
{
  char con_name[255]; 
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
    
     if((type==1)||(type==2)) //task i should precede j, or both combinations are possible
     {   
       //creating a constraint ti + di + dist - tj <= 0     
       SCIP_Real d = tasksToS->at(i)->getDuration();
       SCIP_Real dist;
       if(tasksToS->at(i)->getEndPos().empty()) //if first task has no location, that the travel to following task might take maxDist;
       {
         dist = maxDist;
       }
       else if(tasksToS->at(j)->getStartPos().empty()) //if second task in pair has no location, travel dist is zero, should start immediately
       {
         dist = 0; 
       }
       else
       {        
         dist = DistWrapper::dist(tasksToS->at(i)->getEndPos(),tasksToS->at(j)->getStartPos());
       }
        //two following tasks with no loc
       if((tasksToS->at(i)->getEndPos().empty())&&(tasksToS->at(j)->getStartPos().empty()))
       { 
         dist =0;
       }
       SCIP_Real vals[2]; //array of values
       vals[0] = 1;
       vals[1] = -1; 

       double rhs = -d-dist; 
     
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
     }
     if((type==0)||(type==2)) //task j precede task i, or both combinations are possible
     {
       //creating a constraint tj + dj + dist - ti <= 0

       SCIP_Real dj = tasksToS->at(j)->getDuration();
       SCIP_Real distj;
       if(tasksToS->at(j)->getEndPos().empty()) //if first task has no location, that the travel to following task might take maxDist;
       {
         distj = maxDist;
       }
       else if(tasksToS->at(i)->getStartPos().empty()) //if second task in pair has no location, travel dist is zero, should start immediately
       {
         distj = 0; 
       }
       else
       {        
         distj = DistWrapper::dist(tasksToS->at(j)->getEndPos(),tasksToS->at(i)->getStartPos());
       }
       if((tasksToS->at(j)->getEndPos().empty())&&(tasksToS->at(i)->getStartPos().empty()))
       {
         distj = 0;
       }
       

       SCIP_Real vals3[2]; //array of values
       vals3[0] = 1;
       vals3[1] = -1;  
     
       double rhs2 = -dj-distj;
       SCIP_VAR * vars3[2];
       vars3[0] = tj;
       vars3[1] = ti; 

 
       SCIPsnprintf(con_name, 255, "tddt_%d%d",j,i);

       SCIP_CALL(SCIPcreateConsLinear 	(scip,
		&con2,
		con_name,
		2, //number of variables
		vars3,//&vars,
		vals3,
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
    }
    if(type==1)
    {
      SCIP_CALL( SCIPaddCons(scip, con));
      SCIP_CALL( SCIPreleaseCons(scip, &con));
    }
    if(type==0)
    {
      SCIP_CALL( SCIPaddCons(scip, con2));
      SCIP_CALL( SCIPreleaseCons(scip, &con2));
    }
    if(type==2)
    {
      //create a disjunction
      SCIP_CONS* confinal;
      SCIPsnprintf(con_name, 255, "final_%d%d",i,j);
      SCIP_CONS* arr_final[2];
      arr_final[0] = con;
      arr_final[1] = con2;
      SCIP_CALL(SCIPcreateConsDisjunction(scip,
		&confinal,
		con_name,
		2,
		arr_final,
		NULL, //SCIP_CONS *
		true,
		true,//  	enforce,
		true, //  	check,
		false,//  	local,
		false,// 	modifiable,
		false//  	dynamic 
	)) ;	
      SCIP_CALL( SCIPaddCons(scip, confinal) ); 
      SCIP_CALL( SCIPreleaseCons(scip, &con));
      SCIP_CALL( SCIPreleaseCons(scip, &con2));
      SCIP_CALL( SCIPreleaseCons(scip, &confinal));
    }
  }
  return SCIP_OKAY;
}

/*

Result:
0 - no problem occured
1 - start of j is bigger then end
2 - end of i is smaller then start
*/

SCIP_Retcode ScipUser::editExistingTcons(int i, int j, vector<SCIP_CONS*> * t_con, vector<SCIP_VAR *> * t_var, vector<Task*> * tasksToS, int dist, int * result)
{
  Task * ti = tasksToS->at(i);
  Task * tj = tasksToS->at(j);

  //task i precedes task j, so we would like to modify starting time of task j (more limit interval for task j), we move sj more to left on time axis
  int sj = ti->getDuration() + dist - ti->getStart();
  int xj = tj->getEnd() - tj->getDuration();
  if(sj > xj)
  { 
    //this means, that we need to start after end, this is not feasible
    *result = 1;
    return SCIP_OKAY;
  }

  //everything went successful, we can delete original constraint and create a new one
  SCIP_CALL(SCIPdelCons(scip, t_con->at(j)));
  t_con->at(j) = NULL;
  SCIP_CALL(setOneTcons(j, t_var, t_con, sj, xj));

  /*---------------------------------*/
  //we would like to modify end time of task i (more limit the interval for task i), we move si more to righ on time axis)
  //variable x stands for end time, where task can be executed, it is x = e-d
  int xi = xj - ti->getDuration() - dist;
  int si = ti->getStart();
  if(xi < si)
  {
    //this means, that we need to end before start, this is not feasible
    *result = 2;
    return SCIP_OKAY;
  }
  
  //everything went successful, we can delete original constraint and create a new one
  SCIP_CALL(SCIPdelCons(scip, t_con->at(i)));
  t_con->at(i) = NULL;
  SCIP_CALL(setOneTcons(i, t_var, t_con, si, xi));
  

  return SCIP_OKAY;
}

/* assumption - i is always index of preceding task, j of following task

result:
0 no problem spotted
1 infeasibility
2 fixing of constraint didnt help and it still violates the existing one
3 SCIPcheckCons returned something else then 4 or 5

*/

SCIP_Retcode ScipUser::checkConstraint(SCIP_CONS * con, int i, int j, vector<SCIP_CONS*> * t_con, vector<SCIP_VAR *> * t_var, vector<Task*> * tasksToS, SCIP_Real dist, int runs,  int * ret_val)
{
  //presolve existing problem
  SCIP_CALL(SCIPpresolve(scip));
  int status = SCIPgetStatus(scip); 
  if(status == 11) //11 is infeasible
  {
    *ret_val = 1;
    return SCIP_OKAY;
  }
  else //presolve went well
  {    
    SCIP_SOL * sol = SCIPgetBestSol(scip);
    SCIP_RESULT * res = new SCIP_RESULT();

    //check new constraint again the existing problem
    SCIP_CALL( SCIPcheckCons(scip,
		con, //  	cons,
		sol,//  	sol,
		TRUE,// 	checkintegrality,
		TRUE,//  	checklprows,
		TRUE,//  	printreason,
		res //  	result 
    ));
    //we need to free solution to be able to add the constraint
    SCIP_CALL( SCIPfreeTransform(scip));
        
    int result = *res;
    if(result == 4) //constrain is feasible with existing problem
    {
      SCIP_CALL( SCIPaddCons(scip, con));
      SCIP_CALL( SCIPreleaseCons(scip, &con));
    }
    else if(result == 5) //constraint violates existing problem
    {
      if(runs ==0) //first run
      {
        int * resExistCons;
        *resExistCons = 0;

        SCIP_CALL(editExistingTcons(i, j, t_con, t_var, tasksToS, dist, resExistCons));
        if(*resExistCons == 0)
        {
          //try to check the original constraint again, should proceed smooth
          //recursive call
          checkConstraint(con, i, j, t_con, t_var, tasksToS, dist, 1, ret_val);
        }
        else //infeasibility occured
        {
          *ret_val = 1;
          return SCIP_OKAY;
        }
      }
      else //the constraint was changed, but still it is violated. this should not happen
      {
        *ret_val = 2; 
        return SCIP_OKAY;
      }
    }
    else //this state should not happen, if I understood SCIP_Result properly
    {
      *ret_val = 3;
    }

    //delete pointers
    delete res;
    res = NULL;
    delete sol;
    sol = NULL;
  }
  return SCIP_OKAY;
}

SCIP_Retcode ScipUser::setFinalCons_new(vector<Task*> * tasksToS, vector<SCIP_VAR *> * t_var, vector<vector<int>> * pairs, double maxDist, string filename, vector<SCIP_CONS*> * t_con)
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
         distij = DistWrapper::dist(tasksToS->at(i)->getEndPos(),tasksToS->at(j)->getStartPos());
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
         distji = DistWrapper::dist(tasksToS->at(j)->getEndPos(),tasksToS->at(i)->getStartPos());
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
    }
    if(type==1)
    {
      int * result = new int;
      *result = 0;
      SCIP_CALL(checkConstraint(con, i, j, t_con, t_var, tasksToS, distij,0,result)); 
      
      delete result;
      result = NULL;
    }
    if(type==0)
    {
      SCIP_CALL(SCIPpresolve(scip));
      SCIP_CALL(SCIPprintStatus(scip,NULL)); 
      SCIP_SOL * sol = SCIPgetBestSol(scip);
      SCIP_RESULT * res = new SCIP_RESULT();
      SCIP_Retcode retu = SCIPcheckCons(scip,
		con2, //  	cons,
		sol,//  	sol,
		TRUE,// 	checkintegrality,
		TRUE,//  	checklprows,
		TRUE,//  	printreason,
		res //  	result 
	);

 int a= retu;
cout <<"return code" << a << "\n";

      SCIP_CALL( SCIPfreeTransform(scip));
      int result2 = *res;
cout << "result" << result2 << "\n";
      
      if((result2 == 4)||(result2==5))
      {
        SCIP_CALL( SCIPaddCons(scip, con2));
        SCIP_CALL( SCIPreleaseCons(scip, &con2));
      }
      else
      {
        int temp = cin.get();
        numinfeas++;
      }


      delete res;
      res = NULL;
      delete sol;
      sol = NULL;

    }
    if(type==2)
    {
      SCIP_CALL( setFullConstr(tasksToS, t_var, i, j, maxDist));
    }
  }

  if(!filename.empty())
  {
     results.open (filename,std::ios_base::app);
     results << numinfeas << " ";
  }
  return SCIP_OKAY;
}

SCIP_Retcode ScipUser::setFullConstr(vector<Task*> * tasksToS, vector<SCIP_VAR *> * t_var, int i, int j, double maxDist)
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
         distij = DistWrapper::dist(tasksToS->at(i)->getEndPos(),tasksToS->at(j)->getStartPos());
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
         distji = DistWrapper::dist(tasksToS->at(j)->getEndPos(),tasksToS->at(i)->getStartPos());
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


SCIP_Retcode ScipUser::setFinalCons_preVar(vector<Task*> * tasksToS, vector<SCIP_VAR *> * t_var, vector<vector<int>> * pairs, double maxDist)
{
 
  for(int x=0; x<(int)pairs->size(); x++)
  {

     vector<int> p = pairs->at(x);
     int i = p.at(0);
     int j = p.at(1);
     //int k = p.at(2);
     int type = p.at(3);


     if(type == 2)
     {
       setFullConstr(tasksToS, t_var, i, j, maxDist);
     }
  }
  return SCIP_OKAY;
}


SCIP_Retcode ScipUser::scipSolve(vector<Task*> * tasksToS, SCIP_VAR * vars[], bool * worked, string filename, const int & timeout)
{
  int num_tasks = tasksToS -> size();
  //std::chrono::time_point<std::chrono::system_clock> start, end;
  std::chrono::high_resolution_clock::time_point start, end;
  ofstream results;

  SCIP_Real vals[num_tasks]; //array to save execution times
  start = std::chrono::high_resolution_clock::now();

  //cancel the output to the terminal
  SCIPsetMessagehdlr(scip, NULL);

  if(timeout > 0) {
    SCIP_CALL( SCIPsetRealParam(scip, "limits/time", timeout) );
  }
  
  //adding an offset to objective - produces a segmanation fault - maybe fixed in 3.1
  //SCIP_Real addval = 0.0;
  //SCIP_CALL (SCIPaddObjoffset (scip, addval)); 


	
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
    }
    SCIP_CALL(SCIPgetSolVals(scip,sol, num_tasks, vars, vals)); 
    for(int i=0; i < num_tasks; i++)
    {
      tasksToS->at(i)->setExecTime(vals[i]);
    }
  }	
  results.close();
  return SCIP_OKAY;
}
