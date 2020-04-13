#include<iostream>
#include<math.h>

using namespace std;

class PIDController {

public:
  PIDController(double _dt, double _max, double _min, double _kp, double _kd, double _ki);
  double calculate(double _setpoint, double _pv);

private:
  double dt;
  double max;
  double min;
  double kp;
  double kd;
  double ki;
  double prev_error;
  double integral;
};

PIDController::PIDController(double _dt, double _max, double _min, double _kp, double _kd, double _ki) {
  dt = _dt;
  max = _max;
  min = _min;
  kp = _kp;
  kd = _kd;
  ki = _ki;

}


double PIDController::calculate(double _setpoint, double _pv) {
  double error = _setpoint - _pv;
  
  double proportional_output = kp * error;
  
  integral += error * dt;
  
  double integral_output = ki * integral;
  
  double derivative = (error - prev_error) / dt;
  double derivative_output = kd * derivative;
  
  double total_output = proportional_output + integral_output + derivative_output;
  
  if(total_output > max) {
    total_output = max;
  }
  else if(total_output < min) {
    total_output = min;
  }
  
  prev_error = error;
  
  return total_output;
  
}

















