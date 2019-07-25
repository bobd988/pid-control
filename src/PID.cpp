#include "PID.h"

#include <algorithm>
// using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  _Kp = Kp;
  _Ki = Ki;
  _Kd = Kd;

  _p_error = 0.0;
  _i_error = 0.0;
  _d_error = 0.0;

  // Previous cte.
  _prev_cte = 0.0;

  // Counters.
  _count = 0;
  _error_sum = 0.0;
}

void PID::UpdateError(double cte) {
  // Proportional error.
  _p_error = cte;

  // Integral error.
  _i_error += cte;

  // Diferential error.
  _d_error = cte - _prev_cte;
  _prev_cte = cte;

  _error_sum += cte;
  _count++;

}

double PID::TotalError() {
  return _p_error * _Kp + _i_error * _Ki + _d_error * _Kd;
}

double PID::AverageError() {
  return _error_sum/_count;
}
