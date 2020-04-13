#include<iostream>
#include<math.h>

using namespace std;

class PIDController {

public:

  PIDController(double _dt, double _max, double _min, double _kp, double _kd, double _ki);
  double calculate(double _setPoint, double _input);

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