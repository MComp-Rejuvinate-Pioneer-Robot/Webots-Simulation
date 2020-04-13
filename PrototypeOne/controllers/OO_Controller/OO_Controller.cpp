
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include"PIDController.cpp"
#include<vector>


#define TIME_STEP 64
#define MAX_SPEED 6.4

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

Robot* robot = new Robot();
Motor* motors[4];
DistanceSensor *frontSensors[8];
DistanceSensor *rearSensors[8];

class MotorControl {
public:
  void motorSetup();
  void moveAllMotors(double leftSpeed, double rightSpeed);
  void moveSpecificMotor(int motorIndex, double speed);
  double getMotorSpeed(int motorIndex);
  double getMaxSpeed(int motorIndex);

};

void MotorControl::motorSetup() {
  vector<string> wheelNames = {"back left wheel","back right wheel","front left wheel" ,"front right wheel"};
  
  for(int i = 0; i < 4; i++) {
  motors[i] = robot->getMotor(wheelNames[i]);
  motors[i]->setPosition(INFINITY);
  motors[i]->setVelocity(0.0);
  
  }
}


void MotorControl::moveAllMotors(double leftSpeed, double rightSpeed) {
  motors[0]->setVelocity(leftSpeed);
  motors[1]->setVelocity(rightSpeed);
  motors[2]->setVelocity(leftSpeed);
  motors[3]->setVelocity(rightSpeed);
}

void MotorControl::moveSpecificMotor(int wheelIndex, double speed) {

  motors[wheelIndex]->setVelocity(speed);
}

double MotorControl::getMotorSpeed(int motorIndex) {
  return motors[motorIndex]->getVelocity();
}

double MotorControl::getMaxSpeed(int motorIndex) {
  return motors[motorIndex]->getMaxVelocity();
}


class SonarSensors {
public:
  void frontSensorSetup(int timestep);
  void rearSensorSetup(int timestep);
  double convertToMeters(double value);
  double frontReading(int sensorIndex);
  double rearReading(int sensorIndex);
  double getMaxRange(int sensorIndex);
  double getMinRange(int sensorIndex);

  
};

void SonarSensors::frontSensorSetup(int timestep) {

  vector<string> frontSensorNames = {"so0","so1", "so2", "so3", "so4", "so5", "so6", "so7" };
    
  for(int i = 0; i < 8; i++) {
    frontSensors[i] = robot->getDistanceSensor(frontSensorNames[i]);
    frontSensors[i]->enable(timestep);
  }

}


void SonarSensors::rearSensorSetup(int timestep) {
  vector<string> rearSensorNames = {"so8","so9", "so10", "so11", "so12", "so13", "so14", "so15" };
  
  for(int i = 0; i < 8; i++) {
    rearSensors[i] = robot->getDistanceSensor(rearSensorNames[i]);
    rearSensors[i]->enable(timestep);
  }
}


double SonarSensors::convertToMeters(double value) {
  return ((1000 - value) / 1000) * 5;
}

double SonarSensors::frontReading(int sensorIndex) {
  double reading = frontSensors[sensorIndex]->getValue();
  return convertToMeters(reading);
}

double SonarSensors::rearReading(int sensorIndex) {
  double reading = rearSensors[sensorIndex]->getValue();
  return convertToMeters(reading);
}

double SonarSensors::getMaxRange(int sensorIndex) {
  return frontSensors[sensorIndex]->getMaxValue();
}


double SonarSensors::getMinRange(int sensorIndex) {
  return frontSensors[sensorIndex]->getMinValue();
}

class PioneerBot {
public:
  void componentSetup(int timestep);
  void movement();
  void outputRobotDetails();
private:
  SonarSensors* sonar;
  MotorControl* motorControl;
  // PIDController* PID;

};


void PioneerBot::componentSetup(int timestep) {
  sonar->frontSensorSetup(timestep);
  sonar->rearSensorSetup(timestep);
  motorControl->motorSetup();
  
}

void PioneerBot::movement() {
  PIDController PID = PIDController(0.1, MAX_SPEED, -MAX_SPEED, 0.7, 0.2, 0.5);
  double leftSpeed = MAX_SPEED - 1;
  double rightSpeed = MAX_SPEED - 1;
  
  double input = sonar->frontReading(0);
  double setpoint = 0.2;
  double output = PID.calculate(setpoint, input);
  double newLeftSpeed = leftSpeed + output;
  double newRightSpeed = rightSpeed - output;
  
  
  motorControl->moveAllMotors(newLeftSpeed, newRightSpeed);
  
  bool leftObject = 
    sonar->frontReading(1) < 0.5 ||
    sonar->frontReading(2) < 0.5 ||
    sonar->frontReading(3) < 0.5;
    
  bool rightObject = 
    sonar->frontReading(4) < 0.5 ||
    sonar->frontReading(5) < 0.5 ||
    sonar->frontReading(6) < 0.5;
  
  if(leftObject) {
    motorControl->moveAllMotors(MAX_SPEED, -MAX_SPEED);
  }
}


void PioneerBot::outputRobotDetails() {
  
}


int main(int argc, char **argv) {
  // create the Robot instance.
  // Robot *robot = new Robot();
  
  PioneerBot* pioneer = new PioneerBot();
  
  // nav = &wallFollow;
  // get the time step of the current world.
  
  pioneer->componentSetup(TIME_STEP);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(TIME_STEP) != -1) {
    
    pioneer->movement();

  };


  delete robot;
  return 0;
}
