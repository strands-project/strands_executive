//------------------------------
//guarding to avoid multiple including
#ifndef __DISTWRAPPER_H_INCLUDED__
#define __DISTWRAPPER_H_INCLUDED__

#include <string>
using namespace std;

class DistWrapper
{
  public:
  	///returns time tp travel between two points
  static double dist(string,string);
};

#endif
