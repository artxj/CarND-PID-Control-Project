#include "PID.h"

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  p_error = d_error = i_error = 0;
  prev_cte = 0;
  prev_cte_exists = false;
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PID::UpdateError(double cte) {
  p_error = cte;

  if (prev_cte_exists) {
    d_error = cte - prev_cte;
  } else {
    prev_cte_exists = true;
  }
  prev_cte = cte;

  i_error += cte;
}

double PID::TotalError() const {
  return -Kp * p_error - Kd * d_error - Ki * i_error;
}
