#include "PID.h"
#include <iostream>
#include <math.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID(double const &_p, double const &_i, double const &_d, int _iterations)
: p_error(0.), i_error(0.), d_error(0.)
, coefficients()//Kp(_p), Ki(_i), Kd(_d)
, isInitialized(false)
, cteBias(0.), ctePrevious(0.), cteSum(0.)
, deltaK(), deltaKIndex(-1)
, iterations(_iterations), curIteration(1)
, errorSum(0.), previousErrorSum(-1.)
, condition(false)
{
  coefficients[0] = _p;
  coefficients[1] = _i;
  coefficients[2] = _d;
  deltaK[0] = 0.1;
  deltaK[1] = 0.0002;
  deltaK[2] = 0.7;
  cout<<__FUNCTION__<<"Kp "<<coefficients[0]<<" Ki "<<coefficients[1]<<" Kd "<<coefficients[2]<<endl;
}

PID::~PID() {}


void PID::UpdateError(double cte) {
  if(!isInitialized)
  {
    cout<<__FUNCTION__<<" first call - initialize bias to "<<cte<<endl;
    cteBias = 0.;// cte;
    ctePrevious = 0.;
    isInitialized=true;
  }
  if(curIteration==0)
  {
    //do the twiddle
    if(previousErrorSum == -1.)
    {
      previousErrorSum = errorSum;
      deltaKIndex = 0;
      coefficients[deltaKIndex] += deltaK[deltaKIndex];
    }
    else
    {
      if(condition)
      {
        condition = false;
        if(errorSum < previousErrorSum)
        {
          previousErrorSum = errorSum;
          deltaK[deltaKIndex]*=1.1;
//          coefficients[deltaKIndex] += deltaK[deltaKIndex];
        }
        else
        {
          coefficients[deltaKIndex]+= deltaK[deltaKIndex];
          deltaK[deltaKIndex] *= 0.9;
        }
        deltaKIndex = (deltaKIndex+1)%3;
      }
      else
      {
        if(errorSum < previousErrorSum)
        {
          previousErrorSum = errorSum;
          deltaK[deltaKIndex]*=1.1;
//          coefficients[deltaKIndex] += deltaK[deltaKIndex];
          deltaKIndex = (deltaKIndex+1)%3;
        }
        else
        {
          coefficients[deltaKIndex] -= 2*deltaK[deltaKIndex];
          condition = true;
        }
      }
    }
    errorSum = 0.;
  }
  //adjust the error - we want to stay in the center
  cte-=cteBias;
  cteSum += cte;

  //calculate the errors
  p_error = cte * coefficients[0];
  d_error = (cte - ctePrevious) * coefficients[2];
  i_error = coefficients[1] * cteSum;

  //update the previous cte
  ctePrevious = cte;

//  for debugging purpose only
//  cout<<__FUNCTION__<<"CTE "<<cte<<endl;
}

double PID::TotalError()  {
//  Debugging purpose only
//  cout<<__FUNCTION__<<"P "<<p_error<<" I "<<i_error<<" D "<<d_error<<endl;
  curIteration = (curIteration+1)%iterations;
  if(curIteration == 0)
  {
      cout<<__FUNCTION__<<endl<<endl<<endl<<"P "<<coefficients[0]<<" I "<<coefficients[1]<<" D "<<coefficients[2]<<" ErrorSum "<<errorSum<<endl<<endl<<endl<<endl;
  }
  errorSum += pow(ctePrevious, 2.);
  return -(p_error+d_error+i_error);
}

