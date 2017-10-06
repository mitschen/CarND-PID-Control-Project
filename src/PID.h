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
  double coefficients[3]; //p, i, d
//  double Kp;
//  double Ki;
//  double Kd;
  //Some members
  bool isInitialized;
  double cteBias;
  double ctePrevious;
  double cteSum;
  //twiddle
  double deltaK[3];
  int deltaKIndex;
  int iterations;
  int curIteration;
  double errorSum;
  double previousErrorSum;
  bool condition;
  /*
  * Constructor
  */
  PID(double const &_p, double const &_i, double const &_d, int iterations = 7000);

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
  double TotalError() ;
};

#endif /* PID_H */
