#include "PID.h"
#include <iostream>
#include <math.h>
#include <cassert>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID(PID::TControllerCoefficients const & controllerCfg)
: m_cteSum(0.)
, m_ctePrior(0.)
, m_currentError(0.)
, m_noController(controllerCfg.size())
, m_controller(new TControllerElement [m_noController])
, m_twiddle()
{
  for(int i(0); i < m_noController; i++)
  {
    EControlType const &controllerType(controllerCfg[i].first);
    double const &controllerCoefficient(controllerCfg[i].second);
    m_controller[i].first = controllerCoefficient;
    switch(controllerType)
    {
    case P:
      m_controller[i].second = &PID::p_controller;
      break;
    case I:
      m_controller[i].second = &PID::i_controller;
      break;
    case D:
      m_controller[i].second = &PID::d_controller;
      break;
    }
  }
  cout<<__FUNCTION__<<" with a total of "<<m_noController<< " controllers"<<endl;
}

PID::~PID()
{
  delete [] m_controller;
}


void PID::UpdateError(double const & cte) {
  m_cteSum+=cte;
  m_currentError=0.;
  //iterate over all given controller and sum up the error
  for(int i(0); i<m_noController;i++)
  {
    m_currentError -= (this->*m_controller[i].second)(m_controller[i].first, cte);
  }
  m_ctePrior = cte;
  applyTwiddle();
}

bool PID::setTwiddle(std::vector<double> const &deltas, int noSamples)
{
  if(m_noController != (int)deltas.size())
  {
    return false;
  }
  m_twiddle.reset(deltas, noSamples);
  return true;
}

void PID::applyTwiddle()
{
  if(m_twiddle.is_active)
  {
    int &cnt(m_twiddle.twiddle_cnt);
    m_twiddle.cur_sample++;
    if(m_twiddle.cur_sample == m_twiddle.no_samples)
    {
      bool startNextIteration(false);
      if(m_twiddle.best_Error == -1.)
      {
//        dumpCoefficients();
        m_twiddle.best_Error = m_twiddle.cur_Error;
        //start with the twiddle action
//        m_controller[cnt].first += m_twiddle.delta[cnt];
        startNextIteration = true;
      }
      else if(m_twiddle.best_Error > m_twiddle.cur_Error)
      {
        dumpCoefficients();
        m_twiddle.delta[cnt] *= 1.1;
        m_twiddle.best_Error = m_twiddle.cur_Error;
        m_twiddle.twiddle_second_iteration = false;
        startNextIteration = true;
//        cnt++;
//        m_controller[cnt].first += m_twiddle.delta[cnt];
      }
      else if(m_twiddle.twiddle_second_iteration)
      {
        cout<<"2nd Iteration worse error"<<endl;
        dumpCoefficients();
        m_controller[cnt].first += m_twiddle.delta[cnt];
        m_twiddle.delta[cnt] *= 0.9;
//        cnt++;
//        m_controller[cnt].first += m_twiddle.delta[cnt];
        m_twiddle.twiddle_second_iteration = false;
        startNextIteration = true;
      }
      else
      {
        cout<<"2nd Iteration"<<endl;
        dumpCoefficients();
        m_controller[cnt].first -= 2*m_twiddle.delta[cnt];
        m_twiddle.twiddle_second_iteration = true;
      }
      m_twiddle.cur_Error = 0.;
      m_twiddle.cur_sample = 0;
      if (startNextIteration)
      {
        cout<<"Count "<<cnt<<endl;
        cnt = ((cnt+1)%m_noController);
        m_controller[cnt].first += m_twiddle.delta[cnt];
        cout<<"Starting new iteration"<<endl;
        dumpCoefficients();
      }
    }
    else
    {
      m_twiddle.cur_Error += (m_ctePrior * m_ctePrior);
    }
  }
}

void PID::dumpCoefficients()
{
  cout<<"Twiddle to next iteration - error of current iteration is "<<m_twiddle.cur_Error<<" to "<<m_twiddle.best_Error<<endl;
  for(int i(0); i < m_noController; i++)
  {
    if(m_twiddle.twiddle_cnt==i)
    {
      cout << ">>Coefficient "<<m_controller[i].first<<" with delta "<<m_twiddle.delta[i]<<endl;
    }
    else
    {
      cout << "Coefficient "<<m_controller[i].first<<" with delta "<<m_twiddle.delta[i]<<endl;
    }
  }
  cout<<endl;
}


//TWIDDLE
PID::STwiddleParam::STwiddleParam()
: is_active(false)
, no_samples(0)
, cur_sample(0)
, delta(0)
, best_Error(-1.)
, cur_Error(0.)
, twiddle_cnt(-1)
, twiddle_second_iteration(false)
{}

PID::STwiddleParam::~STwiddleParam()
{}

void PID::STwiddleParam::reset(std::vector<double> const &deltas, int noSamples)
{
  is_active = deltas.size() != 0;
  no_samples = noSamples;
  cur_sample = 0;
  delta = deltas;
  twiddle_cnt = -1;
  best_Error = -1;
  cur_Error=0.;
  twiddle_second_iteration = false;
}

