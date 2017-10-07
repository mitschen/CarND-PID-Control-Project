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
  * @param[IN] the cross track error
  */
  void UpdateError(double const & cte);

  /*
  * Calculate the total PID error.
  */
  double const &TotalError() const
  {
    return m_currentError;
  }
  /*
   * enable the online twiddle optimization
   *
   * @param [IN] deltas that should be applied to the controller coefficients
   * @param [IN] number of measurement samples describing a turn
   * @return true in case that twiddle was activated, false otherwise
   */
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
    bool is_active;                     //twiddle is enabled
    int no_samples;                     //total number of samples before changing twiddle param
    int cur_sample;                     //current number of applied samples
    std::vector<double> delta;          //the deltas for each dimension which should by applied to the coefficients
    double best_Error;                  //best error recorded so far
    double cur_Error;                   //current error recorded in the current iteraiton
    int twiddle_cnt;                    //index of the coefficient currently twiddled
    bool twiddle_second_iteration;      //condition value to reflec the two-level twiddle algorithm
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

  /**
   * do online twiddle algorithm
   * This method is called during @see UpdateError
   * and will only have an impact in case that the client
   * has enabled the twiddle by calling @see setTwiddle
   */
  void applyTwiddle();

  /**
   * dump the resulting coefficients to console
   */
  void dumpCoefficients();

};

#endif /* PID_H */
