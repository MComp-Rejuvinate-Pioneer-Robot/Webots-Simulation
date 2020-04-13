#include"PIDController.hpp"

PIDController::PIDController(double _dt, double _max, double _min, double _kp, double _kd, double _ki) {

  dt = _dt;
  max = _max;
  min = _min;
  kp = _kp;
  kd = _kd;
  ki = _ki;

}


double PIDController::calculate(double _setPoint, double _input) {

  double error = _setPoint - _input;
  
  double proportional_output = kp * error;
  
  
  integral += error * dt;
  
  double integral_output = ki * integral;
  
  double derivative = (error - prev_error) / dt;
  double derivative_output = kd * derivative;
  
  double total_output = proportional_output + integral_output + derivative_output; // gets the sum of all outputs
  
  // in this case checks if the controller output is whithin the Pioneer's speed boundaries
  if(total_output > max) {
    total_output = max;
  }
  else if(total_output < min) {
    total_output = min;
  }
  
  prev_error = error; // update the error
  
  return total_output;
}