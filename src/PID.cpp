#include <cmath>
#include <algorithm>
#include "PID.h"


/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;

  p_error_ = 0;
  d_error_ = 0;
  i_error_ = 0;
  first_step_ = true;
}

void PID::UpdateError(double cte) {
  if (first_step_) {
    d_error_ = 0;
    first_step_ = false;
  }
  else d_error_ = cte - p_error_;
  p_error_ = cte;
  i_error_ += cte;
}

double PID::Response() {
  double response = -(Kp_ * p_error_ + Kd_ * d_error_ + Ki_ * i_error_);
//  if (response > 1) response = std::fmod(response, 1);
//  else if (response < -1) response = std::fmod(response, -1);

  return std::max(-1., std::min(1., response));

}

