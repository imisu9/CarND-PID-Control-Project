#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  
  // Need to figure out how to implement "if needed"
  if (true) {
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
  }
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() {
  /**
   * Initialize twiddle variables
   */
  double dKp = 1.0;
  double dKi = 1.0;
  double dKd = 1.0;
  double tolerance = 0.2; // initialized as shown in the class
  double best_error;
  double curr_error;
  //int iteration = 1;
  
  /**
   * TODO: Calculate and return the total error
   */
  best_error = Kp*p_error + Ki*i_error + Kd*d_error;
  while ((dKp+dKi+dKd) > tolerance) {
    //std::cout << "Iteration: " << iteration
    //          < " best_error: " << best_error << std::endl;
    // Proportional
    Kp += dKp;
    curr_error = Kp*p_error + Ki*i_error + Kd*d_error;
    if (curr_error < best_error) {
      best_error = curr_error;
      dKp *= 1.1;
    } else {
      // Move to the opposite direction
      Kp -= 2*dKp;
      curr_error = Kp*p_error + Ki*i_error + Kd*d_error;
      if (curr_error < best_error) {
        best_error = curr_error;
        dKp *= 1.1;
      } else {
        Kp += dKp;
        dKp *= 0.9;
      }
    }
    
    // Integral
    Ki += dKi;
    curr_error = Kp*p_error + Ki*i_error + Kd*d_error;
    if (curr_error < best_error) {
      best_error = curr_error;
      dKi *= 1.1;
    } else {
      // Move to the opposite direction
      Ki -= 2*dKi;
      curr_error = Kp*p_error + Ki*i_error + Kd*d_error;
      if (curr_error < best_error) {
        best_error = curr_error;
        dKi *= 1.1;
      } else {
        Ki += dKi;
        dKi *= 0.9;
      }
    }
    
    // Differential
    Kd += dKd;
    curr_error = Kp*p_error + Ki*i_error + Kd*d_error;
    if (curr_error < best_error) {
      best_error = curr_error;
      dKd *= 1.1;
    } else {
      // Move to the opposite direction
      Kd -= 2*dKd;
      curr_error = Kp*p_error + Ki*i_error + Kd*d_error;
      if (curr_error < best_error) {
        best_error = curr_error;
        dKd *= 1.1;
      } else {
        Kd += dKd;
        dKd *= 0.9;
      }
    }
  }
  return best_error;
}
