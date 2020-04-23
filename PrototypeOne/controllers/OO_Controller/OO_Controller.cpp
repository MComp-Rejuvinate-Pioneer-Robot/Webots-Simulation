
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include"PIDController.cpp"
#include<vector>
#include<math.h>


#define TIME_STEP 64
#define MAX_SPEED 6.4
#define SPEED_IN_DEGREES 288

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

// class API calls for the robot
Robot* robot = new Robot();
Motor* motors[4];
PositionSensor* motorSensors[4];
DistanceSensor *frontSensors[8];
DistanceSensor *rearSensors[8];


// MOTOR CONTROL CLASS
// this class handles the setup and movement of the motors
class MotorControl {
public:
  void motorSetup();
  
  void moveAllMotors(double leftSpeed, double rightSpeed);
  void moveSpecificMotor(int motorIndex, double speed);
  
  void stopMotors();
  
  void turnLeft(double degrees);
  void turnRight(double right);
  
  double getMotorSpeed(int motorIndex);
  double getMaxSpeed(int motorIndex);
  double calculateRotationTime(double degrees);
};


// MOTOR CONTROL METHODS

// motor setup handles the setup of the Pioneer motors so it can perform movement actions in the simulation
void MotorControl::motorSetup() {
  vector<string> wheelNames = { "back left wheel",
                                  "back right wheel",
                                  "front left wheel",
                                  "front right wheel"};
                                  
  for(int i = 0; i < 4; i++) {
  motors[i] = robot->getMotor(wheelNames[i]);
  motors[i]->setPosition(INFINITY);
  motors[i]->setVelocity(0.0);
  
  }
}

// move all the motors based on the specified speed for the left and right motors
void MotorControl::moveAllMotors(double leftSpeed, double rightSpeed) {
  motors[0]->setVelocity(leftSpeed);
  motors[1]->setVelocity(rightSpeed);
  motors[2]->setVelocity(leftSpeed);
  motors[3]->setVelocity(rightSpeed);
}

// moves a specific motor by a specified speed.
// the motor is determined by the selected index
void MotorControl::moveSpecificMotor(int wheelIndex, double speed) {

  motors[wheelIndex]->setVelocity(speed);
}

void MotorControl::stopMotors() {
  motors[0]->setVelocity(0);
  motors[1]->setVelocity(0);
  motors[2]->setVelocity(0);
  motors[3]->setVelocity(0);
}


// perform a left turn based on the amount of degrees passed into the method
// may need tweaking to make the degrees more accurate
void MotorControl::turnLeft(double degrees) {
   moveAllMotors(-MAX_SPEED, MAX_SPEED);
   double duration = calculateRotationTime(degrees);
   double start = robot->getTime();
   do {
   
     robot->step(TIME_STEP);
     
   } while(robot->getTime() < start + duration);
   stopMotors();
}

// same as left turn method except motors rotate the opposite way
void MotorControl::turnRight(double degrees) {
  moveAllMotors(MAX_SPEED, -MAX_SPEED);
  double duration = calculateRotationTime(degrees);
  double start = robot->getTime();
  do {
  
    robot->step(TIME_STEP);
    
  } while(robot->getTime() < start + duration);
  stopMotors();
}

// returns the current motor speed, useful for visualising speed changes with PID wall follow
double MotorControl::getMotorSpeed(int motorIndex) {
  return motors[motorIndex]->getVelocity();
}

// returns the maximum speed of a motor
double MotorControl::getMaxSpeed(int motorIndex) {
  return motors[motorIndex]->getMaxVelocity();
}

// returns the rotation time of a motor
double MotorControl::calculateRotationTime(double degrees) {
  return abs(degrees) / SPEED_IN_DEGREES;
}
// END MOTOR CONTROL


// POSITION SENSORS
// position sensors represent the wheel encoders of the Pioneer 3-AT
class MotorPositionSensors {
public:
  void sensorSetup(int timestep);
  double getPosition(int currentIndex);

};

// sets up the position sensors so they can be used in the simulation
void MotorPositionSensors::sensorSetup(int timestep) {
  vector<string> wheelSensorNames = { "back left wheel sensor", 
                                      "back right wheel sensor", 
                                      "front left wheel sensor", 
                                      "front right wheel sensor"};
  for(int i = 0; i < 4; i++) {
    motorSensors[i] = robot->getPositionSensor(wheelSensorNames[i]);
    motorSensors[i]->enable(timestep);
    motorSensors[i]->getMotor();
    
  }
}

// returns the current position for the wheel index
double MotorPositionSensors::getPosition(int currentIndex) {
  return motorSensors[currentIndex]->getValue();
}
// END POSITION SENSORS



// SONAR SENSOR CLASS DECLARATION

// class for the ultrasonic sensors
class SonarSensors {
public:
  void frontSensorSetup(int timestep);
  void rearSensorSetup(int timestep);
  
  double convertToMeters(double value);
  
  double frontReading(int sensorIndex);
  double rearReading(int sensorIndex);
  
  double getMaxRange(int sensorIndex);
  double getMinRange(int sensorIndex);
  
  bool leftObject(double threshold);
  bool rightObject(double threshold);
  bool frontObject(double threshold);


};


// SONAR SENSOR METHODS

// sensor setup methods to enable them to be used.
// the sensor names are specific to the robot based on the webots documentation page: Pioneer 3-AT
void SonarSensors::frontSensorSetup(int timestep) {
  
  // FRONT SENSORS LEFT TO RIGHT
  vector<string> frontSensorNames = {"so0","so1", "so2", "so3", "so4", "so5", "so6", "so7" };
    
  for(int i = 0; i < 8; i++) {
    frontSensors[i] = robot->getDistanceSensor(frontSensorNames[i]);
    frontSensors[i]->enable(timestep);
  }

}

// same as front sensor setup, but for the rear 8 sensors
void SonarSensors::rearSensorSetup(int timestep) {
  // REAR SENSORS RIGHT TO LEFT
  vector<string> rearSensorNames = {"so8","so9", "so10", "so11", "so12", "so13", "so14", "so15" };
  
  for(int i = 0; i < 8; i++) {
    rearSensors[i] = robot->getDistanceSensor(rearSensorNames[i]);
    rearSensors[i]->enable(timestep);
  }
}

// meter conversion method was created to help with debugging and to easily visualise the distance from sensors.
double SonarSensors::convertToMeters(double value) {
  return ((1000 - value) / 1000) * 5;
}

// returns the reading of one of the 8 front sensors based on index
double SonarSensors::frontReading(int sensorIndex) {
  double reading = frontSensors[sensorIndex]->getValue();
  return convertToMeters(reading);
}

// same as front reading, but for the rear sensors
double SonarSensors::rearReading(int sensorIndex) {
  double reading = rearSensors[sensorIndex]->getValue();
  return convertToMeters(reading);
}

// returns the maximum range of the sonar sensors in meters
double SonarSensors::getMaxRange(int sensorIndex) {
  double max = frontSensors[sensorIndex]->getMaxValue();
  return convertToMeters(max);
}

// returns the minimum range of the sonar sensors in meters
double SonarSensors::getMinRange(int sensorIndex) {
  double min = frontSensors[sensorIndex]->getMinValue();
  return convertToMeters(min);
}

// the three left side sensors if there is an object based on the threshold distance
bool SonarSensors::leftObject(double threshold) {
  bool left = 
    frontReading(1) < threshold ||
    frontReading(2) < threshold ||
    frontReading(3) < threshold;
  return left;   
}

// checks three right side sensors if there is an object based on the threshold distance
bool SonarSensors::rightObject(double threshold) {
  bool right = 
    frontReading(4) < threshold ||
    frontReading(5) < threshold ||
    frontReading(6) < threshold;
  return right;
}

// checks front two sensors for object based on threshold distance
bool SonarSensors::frontObject(double threshold) {
  bool front =
    frontReading(3) < threshold ||
    frontReading(4) < threshold;
  return front;
}
// END SONAR SENSOR METHODS


// PIONEER BOT CLASS DECLARATION
class PioneerBot {
public:
  void componentSetup(int timestep);
  // void movement();
  void outputRobotDetails();
private:
  SonarSensors* sonar;
  MotorControl* motorControl;
  MotorPositionSensors* motorPos;

};

// setup the sensors and actuators of the pioneer 3-AT
void PioneerBot::componentSetup(int timestep) {
  sonar->frontSensorSetup(timestep);
  sonar->rearSensorSetup(timestep);
  motorPos->sensorSetup(timestep);
  motorControl->motorSetup();
  
}

// output the details of the Pioneer, 
// this includes the max speed, maximum sensor range etc
void PioneerBot::outputRobotDetails() {
  cout << "Motor type in simulation: Rotational" << endl;
  cout << "Max speed: " << motorControl->getMaxSpeed(0) << " Radians per second" << endl;
  cout << "Sensor type: Ultrasonic" << endl;
  cout << "Maximum sensor range: " << sonar->getMaxRange(0) << " meters" << endl;
  cout << "Minimum sensor range: " << sonar->getMinRange(0) << " meters" << endl;
}

// NAVIGATION CLASS
// this class handles the navigation for the Pioneer 
class Navigation {
public:
  void leftWallNav();
  void rightWallNav();
  void multiWallNav();
  void alternatingWallNav();
  void obstacleAvoidance();
private:
  PioneerBot* pioneer;
  SonarSensors* sonar;
  MotorControl* motorControl;
  int obstacleCounter;
  double leftSpeed = MAX_SPEED - 1;
  double rightSpeed = MAX_SPEED - 1;
};// END CLASS DECLARATION

// LEFT WALL NAVIGATION
// this method follows the wall to the robot's left using the sonar sensors
// a PID controller is used to help the robot maintain a set distance from the wall
void Navigation::leftWallNav() {
  PIDController PID = PIDController(0.1, MAX_SPEED, -MAX_SPEED, 0.7, 0.2, 0.5);

  double input = sonar->frontReading(0);
  double threshold = 0.5;
  double setpoint = 0.5;
  double output = PID.calculate(setpoint, input);
  double newLeftSpeed = leftSpeed + output;
  double newRightSpeed = rightSpeed - output;
  
  motorControl->moveAllMotors(newLeftSpeed, newRightSpeed);

  
  if(sonar->leftObject(threshold)) {
    motorControl->moveAllMotors(MAX_SPEED, -MAX_SPEED);
  }
}// END LEFT WALL NAVIGATION


//RIGHT WALL NAVIGATION

void Navigation::rightWallNav() {
  PIDController PID = PIDController(0.1, MAX_SPEED, -MAX_SPEED, 0.7, 0.2, 0.5);

  double input = sonar->frontReading(7);
  double threshold = 0.5;
  double setpoint = 0.5;
  double output = PID.calculate(setpoint, input);
  double newLeftSpeed = leftSpeed - output;
  double newRightSpeed = rightSpeed + output;
  
  motorControl->moveAllMotors(newLeftSpeed, newRightSpeed);
    
  
  if(sonar->rightObject(threshold)) {
    motorControl->moveAllMotors(-MAX_SPEED, MAX_SPEED);

  }
}// END RIGHT WALL NAVIGATION


// MULTIWALL NAVIGATION
// this method handles navigation for multiple walls
// a PID controller is used to maintain the robot's position between two walls
// the PID input changes based on the nearest wall
void Navigation::multiWallNav() {
  
  PIDController PID = PIDController(0.1, MAX_SPEED, -MAX_SPEED, 0.7, 0.2, 0.5);

  double leftInput = sonar->frontReading(0);
  double rightInput = sonar->frontReading(7);
  double setpoint = (leftInput + rightInput) / 2;
  double newLeftSpeed = leftSpeed;
  double newRightSpeed = rightSpeed;
  
  if(leftInput < rightInput) {
    double output = PID.calculate(setpoint, leftInput);
    newLeftSpeed = leftSpeed + output;
    newRightSpeed = rightSpeed - output;
  }
  
  if(rightInput < leftInput) {
    double output = PID.calculate(setpoint, rightInput);
    newLeftSpeed = leftSpeed - output;
    newRightSpeed = rightSpeed + output;
  }

  motorControl->moveAllMotors(newLeftSpeed, newRightSpeed);
  
  double threshold = 0.2;
  
  
  if(sonar->frontObject(threshold) && leftInput < rightInput) {
    // motorControl->moveAllMotors(MAX_SPEED, -MAX_SPEED);
    motorControl->turnRight(45);

  }
  
  if(sonar->frontObject(threshold) && rightInput < leftInput) {
    // motorControl->moveAllMotors(-MAX_SPEED, MAX_SPEED);
    motorControl->turnLeft(45);

  }
  if(sonar->leftObject(threshold)) {
    motorControl->moveAllMotors(MAX_SPEED, -MAX_SPEED);
    // motorControl->moveAllMotorsRight(90);

  }
  if(sonar->rightObject(threshold)) {
    // motorControl->moveAllMotors(-MAX_SPEED, MAX_SPEED);
    motorControl->turnLeft(45);

  }
}// END MULTI WALL NAVIGATION


// ALTERNATING WALL NAV
// dynamically switches between left wall navigation and right wall navigation based on distance from the wall
void Navigation::alternatingWallNav() {
  if(sonar->frontReading(0) < sonar->frontReading(7)) {
    leftWallNav();
  }

  if(sonar->frontReading(7) < sonar->frontReading(0)) {
    rightWallNav();
  }

}

// END ALTERNATING WALL NAV


// SIMPLE OBSTACLE AVOIDANCE MOVEMENT
// when an obstacle is detected,
// the counter is incremented which is then added/subtracted from the motor speed to avoid obstacles

void Navigation::obstacleAvoidance() {

  obstacleCounter = 0;
  double threshold = threshold;  
    
  if(sonar->leftObject(threshold)) {
    obstacleCounter += 1;
    leftSpeed += obstacleCounter;
    rightSpeed -= obstacleCounter;
  }

  if(sonar->rightObject(threshold)) {
    obstacleCounter += 1;
    leftSpeed -= obstacleCounter;
    rightSpeed += obstacleCounter;
  }
   motorControl->moveAllMotors(leftSpeed, rightSpeed);
}

// END SIMPLE OBSTACLE AVOIDANCE MOVEMENT.


int main(int argc, char **argv) {
  // create the Robot instance.
  // Robot *robot = new Robot();
  
  PioneerBot* pioneer = new PioneerBot();
  Navigation nav;
  
  
  pioneer->componentSetup(TIME_STEP);
  pioneer->outputRobotDetails();

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(TIME_STEP) != -1) {
    // nav.leftWallNav();
    // nav.rightWallNav();
    nav.multiWallNav();
    // nav.alternatingWallNav();
    // nav.obstacleAvoidance();

  };


  delete robot;
  return 0;
}
