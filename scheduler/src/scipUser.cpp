#include "scipUser.h"
#include "task.h"
#include "distWrapper.h"

/* scip includes */
#include "objscip/objscip.h"
#include "objscip/objscipdefplugins.h"

#include <vector>

#include <iostream> //TODO:delete this

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

  f= (SCIP_VAR*)NULL;
  pre_var = new vector<SCIP_VAR *>(0,(SCIP_VAR*) NULL); 
  num_preVar = 0;

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

SCIP_VAR * ScipUser::getF() {return f;}

vector<SCIP_VAR*> * ScipUser::getPreVar() {return pre_var;}

SCIP_Retcode ScipUser::fakeVar()
{
  /*creating of fake variable, it is always 1, needed in some constraints*/
   char fn[255];
   SCIPsnprintf(fn, 255, "f");
   SCIP_CALL( SCIPcreateVar(scip,
		&f,
		fn,
		1.0,
		1.0,
		0.0,
		SCIP_VARTYPE_INTEGER,
		true,
		false,
		0, 0, 0, 0, 0));  
   SCIP_CALL(SCIPaddVar(scip, f)); 
   return SCIP_OKAY; 
}

SCIP_Retcode ScipUser::fakeVarReturn(SCIP_VAR * g)
{
  /*creating of fake variable, it is always 1, needed in some constraints*/
   char gn[255];
   SCIPsnprintf(gn, 255, "g");
   SCIP_CALL( SCIPcreateVar(scip,
		&g,
		gn,
		1.0,
		1.0,
		0.0,
		SCIP_VARTYPE_INTEGER,
		true,
		false,
		0, 0, 0, 0, 0)); 
   
   SCIP_CALL(SCIPaddVar(scip, g)); 
   return SCIP_OKAY; 
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
                     1.0,         // objective probably means if this should be included to the minimizing, 1=yes
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

SCIP_Retcode ScipUser::preVar(int i, int j, SCIP_Real low, SCIP_Real up, int * order)
{
  /* add one pre_ij variable (pre_ij = 1 if a task i precede task j) */
   char var_name[255];
   vector<SCIP_VAR*>::iterator it;
   it = pre_var-> begin()+num_preVar;
   SCIPsnprintf(var_name, 255, "pre_i%d_j%d", i,j);
   SCIP_VAR* var;
   SCIP_CALL( SCIPcreateVar(scip,
                     &var,                   // returns new index
                     var_name,               // name
                     low,                    // lower bound
                     up,                    // upper bound
                     0.0,         // objective ??
                     SCIP_VARTYPE_BINARY,   // variable type
                     true,                   // initial
                     false,                  // 
                     0, 0, 0, 0, 0) );
    SCIP_CALL( SCIPaddVar(scip, var) );
    *order = num_preVar;
    num_preVar++;
    pre_var->insert(it,var);  

  
  return SCIP_OKAY;
}

SCIP_Retcode ScipUser::setTcons(vector<Task*> * tasksToS, vector<SCIP_VAR *> * t_var, SCIP_VAR * g)
{
  /* add constraint s<= t + d <=e */
   char con_name[255];
   int num_tasks = tasksToS->size();

   vector<SCIP_CONS*> t_con (num_tasks);
   for (int i = 0; i < num_tasks; i++)
   {
     SCIP_VAR* ti = (SCIP_VAR*)NULL;
     ti = t_var->at(i);
     SCIP_Real d = tasksToS->at(i)->getDuration(); 
     SCIP_Real s = tasksToS->at(i)->getStart();
     SCIP_Real e = tasksToS->at(i)->getEnd();


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
	) );

     SCIP_CONS* con2 = (SCIP_CONS*)NULL;
     SCIPsnprintf(con_name, 255, "e_%d", i);

    SCIP_CALL(SCIPcreateConsLinear (scip,
		&con2,
		con_name,
		1, //number of variables
		vars0,//&vars,
		vals0,
		-SCIP_DEFAULT_INFINITY,//  	lhs,
		e-d,//  	rhs,
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
                false)
    );

    SCIP_CALL( SCIPaddCons(scip, conj) );
    t_con[i] = conj;	
   }
  return SCIP_OKAY;
}


SCIP_Retcode ScipUser::setFinalCons_long(vector<Task*> * tasksToS, vector<SCIP_VAR *> * t_var, SCIP_VAR * g, vector<vector<int>> * pairs)
{
  char con_name[255]; 
  for(int x=0; x<(int)pairs->size(); x++)
  {
     vector<int> p = pairs->at(x);
     int i = p.at(0);
     int j = p.at(1);
     int k = p.at(2);
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
       SCIP_Real dist = DistWrapper::dist(tasksToS->at(i)->getEndPos(),tasksToS->at(j)->getStartPos());

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
       SCIP_Real distj = DistWrapper::dist(tasksToS->at(j)->getEndPos(),tasksToS->at(i)->getStartPos());; 

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

SCIP_Retcode ScipUser::scipSolve(vector<Task*> * tasksToS, SCIP_VAR * vars[], bool * worked)
{
  int num_tasks = tasksToS -> size();

  SCIP_Real vals[num_tasks]; //array to save execution times

  SCIP_CALL( SCIPsolve(scip) );
  //SCIP_CALL( SCIPprintBestSol(scip, NULL, FALSE) );
  SCIP_SOL* sol = SCIPgetBestSol(scip);
  
  if(sol == NULL)
  {
    *worked = false;
    for(int i=0; i < num_tasks; i++)
    {
      tasksToS->at(i)->setExecTime(-1.0);
    }
  }
  else
  {
    *worked = true;
    SCIP_CALL(SCIPgetSolVals(scip,sol, num_tasks, vars, vals)); 
    for(int i=0; i < num_tasks; i++)
    {
      tasksToS->at(i)->setExecTime(vals[i]);
    }
  }	
  return SCIP_OKAY;
}
