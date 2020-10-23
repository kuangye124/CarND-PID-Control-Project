#include "PID.h"
#include <math.h>
#include <iostream>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */

  PID::Kp = Kp_;
  PID::Ki = Ki_;
  PID::Kd = Kd_;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  prev_cte = 0.0;

  // Twiddle parameters
  is_twiddle = false;
  err = 0.0;
  best_err = 999999.0;
  n = 1;
  n_settle = 500;
  dp = {0.01*Kp, 0.01*Ki, 0.01*Kd};
  p = {Kp_, Ki_, Kd_};

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */

  p_error = cte;

  i_error += cte;

  d_error = cte - prev_cte;
  prev_cte = cte;

  //err = err / n;
  n++;

  if (is_twiddle && n>n_settle) {
      err += pow(cte,2);
      for (i=0; i < 3; i++) {
        if (err < best_err) {
          p[i] += dp[i];
          dp[i] *= 1.1;
          best_err = err;
        } else {
          p[i] -= 2 * dp[i];
          if (err < best_err) {
            dp[i] *= 1.1;
            best_err = err;
          } else {
            p[i] += dp[i];
            dp[i] *= 0.9;
          }
        }
      }

  std::cout<<"n: "<<n<<"Err: "<<err<<"Best err:"<<best_err<<std::endl;
  err = 0.0;
  std::cout<<"P: "<<p[0]<<"I: "<<p[1]<<"D: "<<p[2]<<std::endl;
  }

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  double total_error = p[0] * p_error + p[1] * i_error + p[2] * d_error;

  return total_error;  // TODO: Add your total error calc here!
}