//------------------------------
//guarding to avoid multiple including
#ifndef __SCIPUSER_H_INCLUDED__
#define __SCIPUSER_H_INCLUDED__

/* scip includes */
#include "objscip/objscip.h"
#include "objscip/objscipdefplugins.h"

#include "task.h"

#include <vector>

using namespace scip;
using namespace std;

class ScipUser
{
  SCIP* scip;
  SCIP_Retcode catchEr; //catching possible errors in scip
  public:
  ScipUser();
  ~ScipUser();
  SCIP_Retcode getEr();
  SCIP_Retcode tVar(int, vector<SCIP_VAR *> *);
  SCIP_Retcode preVar(vector<SCIP_VAR *> *, int, int);
  SCIP_Retcode setOneTcons(int, vector<SCIP_VAR *> *, vector<SCIP_CONS*> *, int, int);
  SCIP_Retcode setTcons(vector<Task*> *, vector<SCIP_VAR *> *, vector<SCIP_CONS*> *);
  SCIP_Retcode setFinalCons_new(vector<Task*> *, vector<SCIP_VAR *> *, vector<vector<int>> *, double, string, vector<SCIP_CONS*> *, double **);
  SCIP_Retcode setFullConstr(vector<Task*> *, vector<SCIP_VAR *> *, int, int, double, double **);
  SCIP_Retcode setFinalCons_preVar(vector<Task*> *, vector<SCIP_VAR *> *, vector<vector<int>> *, double, double **);
  SCIP_Retcode scipSolve(vector<Task*> *, SCIP_VAR *[],bool*,string, const int & timeout = 0);
};

#endif
