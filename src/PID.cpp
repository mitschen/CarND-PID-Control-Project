#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID(double const &_p, double const &_i, double const &_d)
: p_error(0.), i_error(0.), d_error(0.)
, Kp(_p), Ki(_i), Kd(_d)
, isInitialized(false)
, cteBias(0.), ctePrevious(0.), cteSum(0.)
{
  cout<<__FUNCTION__<<"Kp "<<Kp<<" Ki "<<Ki<<" Kd "<<Kd<<endl;
}

PID::~PID() {}


void PID::UpdateError(double cte) {
  if(!isInitialized)
  {
    cout<<__FUNCTION__<<" first call - initialize bias to "<<cte<<endl;
    cteBias = cte;
    ctePrevious = 0.;
    isInitialized=true;
  }
  //adjust the error - we want to stay in the center
  cte-=cteBias;
  cteSum += cte;

  //calculate the errors
  p_error = cte * Kp;
  d_error = (cte - ctePrevious) * Kd;
  i_error = Ki * cteSum;

  //update the previous cte
  ctePrevious = cte;

//  for debugging purpose only
//  cout<<__FUNCTION__<<"CTE "<<cte<<endl;
}

double PID::TotalError() const {
//  Debugging purpose only
//  cout<<__FUNCTION__<<"P "<<p_error<<" I "<<i_error<<" D "<<d_error<<endl;
  return -(p_error+d_error+i_error);
}

