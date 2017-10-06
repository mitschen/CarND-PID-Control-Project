#ifndef PID_H
#define PID_H

#include <utility>
#include <vector>

class PID {

public:
  //Specification of the controller type
  enum EControlType
  {
    P,
    I,
    D
  };
  //controller configuration types
  typedef std::pair<EControlType, double> TControlCoefficient;
  typedef std::vector<TControlCoefficient> TControllerCoefficients;

  /*
  * Constructor
  */
  PID(PID::TControllerCoefficients const & controllerCfg);

  /*
  * Destructor.
  */
  virtual ~PID();
  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double const & cte);

  /*
  * Calculate the total PID error.
  */
  double const &TotalError() const
  {
    return m_currentError;
  }

  bool setTwiddle(std::vector<double> const &deltas, int noSamples = 8000);
private:
  //MEMBERS AND TYPEDEFS
  typedef double (PID::*controlFct)(double const &coefficient, double const &cte);
  typedef std::pair<double, controlFct> TControllerElement;


  double m_cteSum;                        //sum of all cte received so far
  double m_ctePrior;                      //ctePrior
  double m_currentError;                  //error calculated for this round
  int m_noController;                     //number of controllers
  TControllerElement * m_controller;      //controllers itself
  struct STwiddleParam
  {
    STwiddleParam();
    virtual ~STwiddleParam();
    void reset(std::vector<double> const &deltas, int noSamples);

    //MEMBER
    bool is_active;
    int no_samples;
    int cur_sample;
    std::vector<double> delta;
    double best_Error;
    double cur_Error;
    int twiddle_cnt;
    bool twiddle_second_iteration;
  } m_twiddle;
  /**
   * calculate the p-related part of error
   */
  double p_controller(double const &coefficient, double const &cte)
  {
    return cte*coefficient;
  }
  /**
   * calculate the i-related part of error
   */
  double i_controller(double const &coefficient, double const &cte)
  {
    return m_cteSum*coefficient;
  }
  /**
   * calculate the d-related part of error
   */
  double d_controller(double const &coefficient, double const &cte)
  {
    return (cte-m_ctePrior)*coefficient;
  }

  void applyTwiddle();

  void dumpCoefficients();

};

#endif /* PID_H */
