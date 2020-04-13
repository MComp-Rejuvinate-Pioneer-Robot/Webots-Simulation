// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include<vector>
#include<algorithm>
#include"PIDController.cpp"

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

#define TIME_STEP 64
#define MAX_SPEED 6.4

Robot *robot = new Robot();
DistanceSensor *frontSensors[8];
Motor *wheels[4];


void frontSensorSetup(int timestep) {

  vector<string> frontSensorNames = {"so0","so1", "so2", "so3", "so4", "so5", "so6", "so7" };
  
  //BACK SENSORS LEFT TO RIGHT (S015 So14 So13 So12 So11 So10 So9 So8)
  
  for(int i = 0; i < 8; i++) {
    frontSensors[i] = robot->getDistanceSensor(frontSensorNames[i]);
    frontSensors[i]->enable(timestep);
  }
  
}

void motorSetup() {
  vector<string> wheelNames = {"back left wheel","back right wheel","front left wheel" ,"front right wheel"};
  
  for(int i = 0; i < 4; i++) {
  wheels[i] = robot->getMotor(wheelNames[i]);
  wheels[i]->setPosition(INFINITY);
  wheels[i]->setVelocity(0.0);
  
  }
}

// returns the reading of the sensor, current index parameter represents which sensor (0-8 = front sensors, 9 - 15 = rear sensors)
// converts the sensor value to meters
double sensorVal(int currentIndex) {

   double sensorData = frontSensors[currentIndex]->getValue();
   return ((1000 - sensorData) / 1000) * 5;
}

void moveAllMotors(double leftSpeed, double rightSpeed) {
  wheels[0]->setVelocity(leftSpeed);
  wheels[1]->setVelocity(rightSpeed);
  wheels[2]->setVelocity(leftSpeed);
  wheels[3]->setVelocity(rightSpeed);
}

void moveSpecificMotor(int wheelIndex, double speed) {

  wheels[wheelIndex]->setVelocity(speed);
}


void leftWallFollow() {
  double leftSpeed = 4;
  double rightSpeed = 4;
  
  
  if(sensorVal(0) > 0.5) {
   
    rightSpeed += 2;
    // leftSpeed -= 2;
  }
  if(sensorVal(0) < 0.5) {
    leftSpeed += 2;
    // rightSpeed -= 2;
    
  }
  moveAllMotors(leftSpeed, rightSpeed);
}


void PIDLeftWallFollow() {
  PIDController pid = PIDController(0.1, MAX_SPEED, -MAX_SPEED, 0.7, 0.2, 0.5);
  // cout << wheels[0]->getMaxVelocity() << endl;
  double leftSpeed = MAX_SPEED - 1;
  double rightSpeed = MAX_SPEED - 1;
  
  double input = sensorVal(0);
  double setpoint = 0.2;
  
  double output = pid.calculate(setpoint, input);
  
  double newLeftSpeed = leftSpeed + output;
  double newRightSpeed = rightSpeed - output;
  
  
  moveAllMotors(newLeftSpeed, newRightSpeed);
  
  bool leftObject = 
    sensorVal(1) < 0.5 ||
    sensorVal(2) < 0.5 ||
    sensorVal(3) < 0.5;
    
  bool rightObject = 
    sensorVal(4) < 0.5 ||
    sensorVal(5) < 0.5 ||
    sensorVal(6) < 0.5;
  
  if(leftObject) {
    moveAllMotors(MAX_SPEED, -MAX_SPEED);
  }
  
}


void PIDRightWallFollow() {
  PIDController pid = PIDController(0.1, MAX_SPEED, -MAX_SPEED, 2, 0.5, 0.5);
  // cout << wheels[0]->getMaxVelocity() << endl;
  double leftSpeed = MAX_SPEED - 1;
  double rightSpeed = MAX_SPEED - 1;
  
  double input = sensorVal(7);
  double setpoint = 0.2;
  
  double output = pid.calculate(setpoint, input);
  
  double newLeftSpeed = leftSpeed - output;
  double newRightSpeed = rightSpeed + output;
  
  
  moveAllMotors(newLeftSpeed, newRightSpeed);
    
  bool rightObject = 
    sensorVal(4) < 0.5 ||
    sensorVal(5) < 0.5 ||
    sensorVal(6) < 0.5;
  
  if(rightObject) {
    moveAllMotors(-MAX_SPEED, MAX_SPEED);
  }
}


void PIDBothWallFollow() {
  if(sensorVal(0) < sensorVal(7)) {
    PIDLeftWallFollow();
  }
  else if(sensorVal(7) < sensorVal(0)) {
    PIDRightWallFollow();
  }
}

int main(int argc, char **argv) {

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();
  
  frontSensorSetup(timeStep);
  motorSetup();
    
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    
    // PIDLeftWallFollow();
    // PIDRightWallFollow();
    PIDBothWallFollow();
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
