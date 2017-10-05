#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  //Some members
  bool isInitialized;
  double cteBias;
  double ctePrevious;
  double cteSum;
  /*
  * Constructor
  */
  PID(double const &_p, double const &_i, double const &_d);

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError() const;
};

#endif /* PID_H */
