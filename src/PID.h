#ifndef PID_H
#define PID_H
#include <uWS/uWS.h>

class PID {
 public:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
  
 /**
  * Twiddle
  */
  double p[3];
  double dp[3];

  double best_err;
  double total_error;
  
  int flag;
  int pid_index;
  
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();
  
  /**
  * Restart the simulator for twiddle
  */
  void Restart(uWS::WebSocket<uWS::SERVER> ws);
  
  /**
   * Twiddle for tuning the hyper parameter
   */
  void Twiddle(double total_error);

 private:

};

#endif  // PID_H