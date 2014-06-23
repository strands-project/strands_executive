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
  SCIP_Retcode setTcons(vector<Task*> *, vector<SCIP_VAR *> *);
  SCIP_Retcode setFinalCons(vector<Task*> *, vector<SCIP_VAR *> *, vector<vector<int>> *, double);
  SCIP_Retcode scipSolve(vector<Task*> *, SCIP_VAR *[],bool*,string);
};

#endif
