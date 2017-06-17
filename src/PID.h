#ifndef PID_H
#define PID_H

#include <deque>

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

  /*
  * Twiddle Coefficients
  */
  std::deque<double> p;
  std::deque<double> dp;

  /*
  * Twiddle State
  * 0 - Twiddle Kp / 1 - Twiddle Ki / 2 - Twiddle Kd
  * twiddle_update is 0 if first update and 1 if 2nd update for each coefficient
  */
  int twiddle_state;
  int twiddle_update;

  /*
  * best error value captured by pid controller
  */
  double best_err;


  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Perform Twiddle
  */
  void twiddle(double cte);
};

#endif /* PID_H */
