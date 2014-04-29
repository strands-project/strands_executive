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
  SCIP_VAR * f; //pointer to fake variable, we need to have it global to some error, probably internal SCIP
  vector<SCIP_VAR*> * pre_var;
  int num_preVar;
  public:
  ScipUser();
  ~ScipUser();
  SCIP_Retcode getEr();
  SCIP_VAR * getF();
  vector<SCIP_VAR*> * getPreVar();
  SCIP_Retcode fakeVar();
  SCIP_Retcode fakeVarReturn(SCIP_VAR * g);
  SCIP_Retcode tVar(int, vector<SCIP_VAR *> *);
  SCIP_Retcode preVar(int, int, SCIP_Real, SCIP_Real, int *);
  SCIP_Retcode setTcons(vector<Task*> *, vector<SCIP_VAR *> *, SCIP_VAR *);
  SCIP_Retcode setFinalCons_long(vector<Task*> *, vector<SCIP_VAR *> *, SCIP_VAR *, vector<vector<int>> *);
  SCIP_Retcode scipSolve(vector<Task*> *, SCIP_VAR *[],bool*);
};

#endif
