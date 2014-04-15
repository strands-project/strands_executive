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
     SCIPsnprintf(con_name, 255, "se_%d", i);

     SCIP_CALL(SCIPcreateConsVarbound 	(scip,
		&con,
		con_name,
		ti,	//variable x
		g,      //biding variable y
		d,      //constant
		s,      //left side of eq
		e,	//right side of eq
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
    SCIP_CALL( SCIPaddCons(scip, con) );
    t_con[i] = con;	
   }
  return SCIP_OKAY;
}

SCIP_Retcode ScipUser::setLeftCons(vector<Task*> * tasksToS, vector<SCIP_VAR *> * t_var, int i, int j, SCIP_VAR * g, SCIP_CONS* con)
{
/* add constraint ti + di + dist - tj <= 0 if preij == 1*/
     char con_name[255];   
     SCIP_VAR* ti;
     SCIP_VAR* tj;

     
     ti = t_var->at(i);
     tj = t_var->at(j);

     //creating a constraint ti + di + dist - tj <= 0     
     SCIP_Real d = tasksToS->at(i)->getDuration();
     SCIP_Real dist = DistWrapper::dist(tasksToS->at(i)->getEndPos(),tasksToS->at(j)->getStartPos());

     SCIP_Real vals[4]; //array of values
     vals[0] = 1;
     vals[1] = d;
     vals[2] = dist;
     vals[3] = -1;  
     
     SCIP_VAR * vars[4]; //array of variables
     vars[0] = ti;
     vars[1] = g;
     vars[2] = g;
     vars[3] = tj; 
 
     SCIPsnprintf(con_name, 255, "tddt_%d%d", i,j);

     SCIP_CALL(SCIPcreateConsLinear 	(scip,
		&con,
		con_name,
		4, //number of variables
		vars,//&vars,
		vals,
		-SCIP_DEFAULT_INFINITY,//  	lhs,
		0.0,//  	rhs,
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
  return SCIP_OKAY;
}

SCIP_Retcode ScipUser::setRightCons(int i, int j, int k, double v, SCIP_CONS* con2)
{
 //creating a constraint preij == 1

     char con_name[255];   
     SCIP_VAR * vars2[1];
     vars2[0] = pre_var->at(k);

     SCIP_Real vals2[1];
     vals2[0] = 1.0;

     SCIPsnprintf(con_name, 255, "pre_%d%d", i,j);
     SCIP_CALL(SCIPcreateConsLinear 	(scip,
		&con2,
		con_name,
		1, //number of variables
		vars2,//&vars,
		vals2,
		v,//  	lhs,
		v,//  	rhs,
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

  return SCIP_OKAY;
}

SCIP_Retcode ScipUser::setConjCons(int i, int j, SCIP_CONS * first, SCIP_CONS * second, SCIP_CONS * final)
{
  //create a conjunction
  char con_name[255];   
  SCIPsnprintf(con_name, 255, "jun_%d%d", i,j);
  SCIP_CONS* arr_jun[2];
  arr_jun[0] = first;
  arr_jun[1] = second;
  SCIP_CALL(SCIPcreateConsConjunction( scip,
                &final,
		con_name,
                2,
                arr_jun,
                true,
                true,
                false,
                false,
                false)
    );
  return SCIP_OKAY;
}

SCIP_Retcode ScipUser::setFinalCons(vector<Task*> * tasksToS, vector<SCIP_VAR *> * t_var, SCIP_VAR * g, vector<vector<int>> * pairs)
{
  SCIP_Retcode err;
  char con_name[255];  

  SCIP_CONS* conL1 = new SCIP_CONS();
  SCIP_CONS* conR1 = new SCIP_CONS();
  SCIP_CONS* conF1 = new SCIP_CONS();

  SCIP_CONS* conL2 = new SCIP_CONS();
  SCIP_CONS* conR2 = new SCIP_CONS();
  SCIP_CONS* conF2 = new SCIP_CONS();

  SCIP_CONS* confinal = new SCIP_CONS();

  int i=0;
  int j=1;
  int k=0;
  //create (ti+di+dist-tj)*preij <= 0
  err = setLeftCons(tasksToS, t_var, i, j, g, conL1);
  if (err != SCIP_OKAY)
    return err; 

  err = setRightCons(i, j, k, 1.0, conR1);
  if (err != SCIP_OKAY)
    return err;

  err = setConjCons(i, j, conL1, conR1, conF1);
  if (err != SCIP_OKAY)
    return err;

  //create (tj+dj+dist-ti)*(1-preij) <= 0
  err = setLeftCons(tasksToS, t_var, j, i, g, conL2);
  if (err != SCIP_OKAY)
    return err; 

  err = setRightCons(j, i, k, 0.0, conR2);
  if (err != SCIP_OKAY)
    return err;

  err = setConjCons(j, i, conL2, conR2, conF2);
  if (err != SCIP_OKAY)
    return err;

  //finally, create disjunction of these two conditions
  
  SCIPsnprintf(con_name, 255, "final_%d%d",i,j);
  SCIP_CONS* arr_final[2];
  arr_final[0] = conF1;
  arr_final[1] = conF2;
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
  if (err != SCIP_OKAY)
    return err;


  //add disjunction to the problem
  //SCIP_CALL( SCIPaddCons(scip, conR1) );
  if (err != SCIP_OKAY)
    return err;

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

     SCIP_VAR* ti;
     SCIP_VAR* tj;
   
     ti = t_var->at(i);
     tj = t_var->at(j);

     //creating a constraint ti + di + dist - tj <= 0     
     SCIP_Real d = tasksToS->at(i)->getDuration();
     SCIP_Real dist = DistWrapper::dist(tasksToS->at(i)->getEndPos(),tasksToS->at(j)->getStartPos());

     SCIP_Real vals[4]; //array of values
     vals[0] = 1;
     vals[1] = d;
     vals[2] = dist;
     vals[3] = -1;  
     
     SCIP_VAR * vars[4];
     vars[0] = ti;
     vars[1] = g;
     vars[2] = g;
     vars[3] = tj; 
 

     SCIP_CONS* con;
     SCIPsnprintf(con_name, 255, "tddt_%d%d", i,j);

     SCIP_CALL(SCIPcreateConsLinear 	(scip,
		&con,
		con_name,
		4, //number of variables
		vars,//&vars,
		vals,
		-SCIP_DEFAULT_INFINITY,//  	lhs,
		0.0,//  	rhs,
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

//creating a constraint preij == 1
     SCIP_VAR * vars2[1];
     vars2[0] = pre_var->at(k);

     SCIP_Real vals2[1];
     vals2[0] = 1.0;

     SCIP_CONS* con2;
     SCIPsnprintf(con_name, 255, "pre_%d%d", i,j);

     SCIP_CALL(SCIPcreateConsLinear 	(scip,
		&con2,
		con_name,
		1, //number of variables
		vars2,//&vars,
		vals2,
		1.0,//  	lhs,
		1.0,//  	rhs,
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
    SCIPsnprintf(con_name, 255, "jun_%d%d", i,j);
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

    //creating a constraint tj + dj + dist - ti <= 0

      SCIP_Real dj = tasksToS->at(j)->getDuration();
     SCIP_Real distj = DistWrapper::dist(tasksToS->at(j)->getEndPos(),tasksToS->at(i)->getStartPos());; //TODO: call dist function

     SCIP_Real vals3[4]; //array of values
     vals3[0] = 1;
     vals3[1] = dj;
     vals3[2] = distj;
     vals3[3] = -1;  
     
     SCIP_VAR * vars3[4];
     vars3[0] = tj;
     vars3[1] = g;
     vars3[2] = g;
     vars3[3] = ti; 

     SCIP_CONS* con3;
     SCIPsnprintf(con_name, 255, "tddt_%d%d",j,i);

     SCIP_CALL(SCIPcreateConsLinear 	(scip,
		&con3,
		con_name,
		4, //number of variables
		vars3,//&vars,
		vals3,
		-SCIP_DEFAULT_INFINITY,//  	lhs,
		0.0,//  	rhs,
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

//creating a constraint preij == 0
     SCIP_VAR * vars4[1];
     vars4[0] = pre_var->at(k);

     SCIP_Real vals4[1];
     vals4[0] = 1.0;

     SCIP_CONS* con4;
     SCIPsnprintf(con_name, 255, "pre_%d%d",j,i);

     SCIP_CALL(SCIPcreateConsLinear 	(scip,
		&con4,
		con_name,
		1, //number of variables
		&vars4[0],//&vars,
		&vals4[0],
		0.0,//  	lhs,
		0.0,//  	rhs,
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
    SCIP_CONS* conj2;
    SCIPsnprintf(con_name, 255, "jun_%d%d",j,i);
    SCIP_CONS* arr_jun2[2];
    arr_jun2[0] = con3;
    arr_jun2[1] = con4;
    SCIP_CALL(SCIPcreateConsConjunction( scip,
                &conj2,
		con_name,
                2,
                arr_jun2,
                true,
                true,
                false,
                false,
                false)
    );

    //create a disjunction
    SCIP_CONS* confinal;
    SCIPsnprintf(con_name, 255, "final_%d%d",i,j);
    SCIP_CONS* arr_final[2];
    arr_final[0] = conj;
    arr_final[1] = conj2;
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
  }
  return SCIP_OKAY;
}

SCIP_Retcode ScipUser::scipSolve(vector<Task*> * tasksToS, SCIP_VAR * vars[], bool * worked)
{
  int num_tasks = tasksToS -> size();

  SCIP_Real vals[num_tasks]; //array to save execution times

  SCIP_CALL( SCIPsolve(scip) );
  SCIP_CALL( SCIPprintBestSol(scip, NULL, FALSE) );
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
