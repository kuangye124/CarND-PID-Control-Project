#ifndef PID_H
#define PID_H
#include <vector>

class PID {
 public:
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

  //std::vector<double> Twiddle(double tolerance);

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  // for total error calculating
  double prev_cte;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  // twiddle coefficients
  double err, best_err;
  std::vector<double> dp, p;
  int i, n, n_settle;
  bool is_twiddle;
};

#endif  // PID_H