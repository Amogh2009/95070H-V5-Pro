/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\abhis                                            */
/*    Created:      Mon Jul 18 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// LeftFront            motor         1               
// RightFront           motor         13              
// LeftBack             motor         15              
// RightBack            motor         11              
// RightLift            motor         5               
// Inertial             inertial      21              
// OldbackPiston        digital_out   D               
// Sporklift            motor         19              
// RightMiddle          motor         14              
// ClampSolenoid        digital_out   A               
// IntakeRoller         motor         6               
// Flywheel1            motor         12              
// LeftMiddle           motor         20              
// Flywheel2            motor         9               
// Indexer              motor         10              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>
#include <string>

using namespace vex;
// A global instance of competition
competition Competition;
//Function for determining whether input is positive, negative, or 0

int autonselect = 1;
int numOfAutons = 7;

int getSign (double inputValue) {
  if (inputValue > 0){
    return 1;
  }
  else if (inputValue < 0){
    return -1;
  }
  else return 0;
}

/*Our code uses PID, a control loop used to help the robot move efficiently and accurately
without overshooting its target position. PID takes in input based on the sensors
in the V5 Motors and uses a function to output the target speed for the motors. The "P" in PID
stands for proportional. It makes the motors move based on the distance to the target value.
The "I" in PID stands for integral. It calculates how far the motors have already moved to
give it a little push when proportional control cannot get the robot to its final destination.
The "D" in PID stands for derivative. The derivative calculates how fast the robot has been
accelerating and slows it down if it has been accelerating too rapidly. Combined, these
features create a powerful control loop that keeps our robot's performance consistently high.*/
 
//PID to make the robot drive a certain distance during the autonomous period
void PID (double kP, double kI, double kD, double maxIntegral, double tolerance, double maximumSpeed, double minimumSpeed, double target){
  double error = target;
  double derivative = 0;
  double integral = 0;
  double LastError=error;
  double total = 0;
  LeftBack.setPosition(0, turns);
  Inertial.setRotation(0, degrees);
  while(fabs(tolerance)<fabs(error)){
    LeftBack.spin(forward);
    RightBack.spin(forward);
    LeftFront.spin(forward);
    RightFront.spin(forward);
    LeftMiddle.spin(forward);
    RightMiddle.spin(forward);
    double SensorValue = LeftBack.position(turns)*3.25*5/3*M_PI;
    error = target - SensorValue;
    integral = integral + error;
    if(fabs(integral)>fabs(maxIntegral)){
      integral=getSign(integral)*maxIntegral;
    }
    derivative = error-LastError;
    LastError = error;
    total = kP*error + kI*integral + kD*derivative;
    double amountOff = Inertial.rotation(degrees);
    if(-1 < amountOff < 1){
      amountOff = 0;
    }
    if(fabs(total) > fabs(maximumSpeed)){
      LeftBack.setVelocity(getSign(total)*maximumSpeed - 0.5*amountOff, percent);
      RightBack.setVelocity(getSign(total)*maximumSpeed + 0.5*amountOff, percent);
      LeftFront.setVelocity(getSign(total)*maximumSpeed - 0.5*amountOff, percent);
      RightFront.setVelocity(getSign(total)*maximumSpeed + 0.5*amountOff, percent);
      LeftMiddle.setVelocity(getSign(total)*maximumSpeed + 0.5*amountOff, percent);
      RightMiddle.setVelocity(getSign(total)*maximumSpeed + 0.5*amountOff, percent);
    }
    else if(fabs(total) < fabs(minimumSpeed)){
      LeftBack.setVelocity(getSign(total)*minimumSpeed - 0.5*amountOff, percent);
      RightBack.setVelocity(getSign(total)*minimumSpeed + 0.5*amountOff, percent);
      LeftFront.setVelocity(getSign(total)*minimumSpeed - 0.5*amountOff, percent);
      RightFront.setVelocity(getSign(total)*minimumSpeed + 0.5*amountOff, percent);
      LeftMiddle.setVelocity(getSign(total)*minimumSpeed + 0.5*amountOff, percent);
      RightMiddle.setVelocity(getSign(total)*minimumSpeed + 0.5*amountOff, percent);
    }
    else{
      LeftBack.setVelocity(total - 0.5*amountOff, percent);
      RightBack.setVelocity(total + 0.5*amountOff, percent);
      LeftFront.setVelocity(total - 0.5*amountOff, percent);
      RightFront.setVelocity(total + 0.5*amountOff,percent);
      LeftMiddle.setVelocity(total + 0.5*amountOff,percent);
      RightMiddle.setVelocity(total + 0.5*amountOff,percent);
    }
  }
  LeftBack.stop(brake);
  RightBack.stop(brake);
  RightFront.stop(brake);
  LeftFront.stop(brake);
  RightMiddle.stop(brake);
  LeftMiddle.stop(brake);
}
//Void that controls the drivetrain based on inputs from the joysticks

int speedFactor = 1;

void setStopping(vex::brakeType stoppingType) {
  LeftFront.setStopping(stoppingType);
  LeftBack.setStopping(stoppingType);
  LeftMiddle.setStopping(stoppingType);
  RightFront.setStopping(stoppingType);
  RightBack.setStopping(stoppingType);
  RightMiddle.setStopping(stoppingType);
}

void setVelocity(int velocity) {
  LeftFront.setVelocity(velocity, percent);
  LeftBack.setVelocity(velocity, percent);
  LeftMiddle.setVelocity(velocity, percent);
  RightFront.setVelocity(velocity, percent);
  RightBack.setVelocity(velocity, percent);
  RightMiddle.setVelocity(velocity, percent);
}

typedef int turntype;
const turntype left = 1;
const turntype right = 0;


void turn(turntype direction, int rotation) {
  LeftFront.spinFor(direction ? reverse : forward, rotation, degrees, false);
  LeftBack.spinFor(direction ? reverse : forward, rotation, degrees, false);
  RightFront.spinFor(direction ? forward : reverse, rotation, degrees, false);
  RightBack.spinFor(direction ? forward : reverse, rotation, degrees, true);
}

void move(vex::directionType direction, int rotation) {
  LeftFront.spinFor(direction, rotation, degrees, false);
  LeftBack.spinFor(direction, rotation, degrees, false);
  RightFront.spinFor(direction, rotation, degrees, false);
  RightBack.spinFor(direction, rotation, degrees, true);
}

void platformMode() {
  if(Controller1.ButtonX.pressing()){
    speedFactor = 6;
    LeftFront.setStopping(hold);
    LeftBack.setStopping(hold);
    RightFront.setStopping(hold);
    RightBack.setStopping(hold);
    RightMiddle.setStopping(hold);
    LeftMiddle.setStopping(hold);
  }
  else {
    speedFactor = 1;
    LeftFront.setStopping(coast);
    LeftBack.setStopping(coast);
    RightFront.setStopping(coast);
    RightBack.setStopping(coast);
    RightMiddle.setStopping(coast);
    LeftMiddle.setStopping(coast);
  }
}

void goSlow(){
  if(Controller1.ButtonX.pressing()){
    LeftBack.spin(forward, 50, percent);
    RightBack.spin(forward, 50, percent);
    LeftFront.spin(forward, 50, percent);
    RightFront.spin(forward, 50, percent);
    RightMiddle.spin(forward, 50, percent);
    LeftMiddle.spin(forward, 50, percent);
  }
  else if(Controller1.ButtonB.pressing()){
    LeftBack.spin(reverse, 50, percent);
    RightBack.spin(reverse, 50, percent);
    LeftFront.spin(reverse, 50, percent);
    RightFront.spin(reverse, 50, percent);
    RightMiddle.spin(reverse, 50, percent);
    LeftMiddle.spin(reverse, 50, percent);
  }
  else if(Controller1.ButtonY.pressing()){
    LeftBack.spin(reverse, 50, percent);
    RightBack.spin(forward, 50, percent);
    LeftFront.spin(reverse, 50, percent);
    RightFront.spin(forward, 50, percent);
    RightMiddle.spin(forward, 50, percent);
    LeftMiddle.spin(forward, 50, percent);
  }
  else if(Controller1.ButtonA.pressing()){
    LeftBack.spin(forward, 50, percent);
    RightBack.spin(reverse, 50, percent);
    LeftFront.spin(forward, 50, percent);
    RightFront.spin(reverse, 50, percent);
    RightMiddle.spin(reverse, 50, percent);
    LeftMiddle.spin(reverse, 50, percent);
  }
  else{
    RightBack.stop(hold);
    RightFront.stop(hold);
    LeftBack.stop(hold);
    LeftFront.stop(hold);
    LeftMiddle.stop(hold);
    RightMiddle.stop(hold);
  }
}

void simpleDrive(){
  double forwardAmount = Controller1.Axis3.position();
  double turnAmount = Controller1.Axis1.position(); //Axis 4 for unified joystick
  
  RightFront.spin(forward, (forwardAmount-turnAmount) / speedFactor, percent);
  RightBack.spin(forward, (forwardAmount-turnAmount) / speedFactor, percent);
  LeftFront.spin(forward, (forwardAmount+turnAmount) / speedFactor, percent);
  LeftBack.spin(forward, (forwardAmount+turnAmount) / speedFactor, percent);
}
//Void that controls the movement of the 4-bar lift
void armLift(){
  if (Controller1.ButtonL2.pressing()) {
    RightLift.setVelocity(90, percent);
    RightLift.spin(forward);
  }
  else if (Controller1.ButtonL1.pressing()){
    RightLift.setVelocity(90, percent);
    RightLift.spin(reverse);
  }
  else{
    RightLift.setStopping(hold);
    RightLift.stop();
  }
}

void intakeRollerMovement() {
  if(Controller1.ButtonR1.pressing()){
    /*Clamp.setVelocity(200,percent);
    Clamp.spin(forward);*/
    IntakeRoller.setVelocity(100, percent);
    IntakeRoller.spin(forward);
  }
  else if(Controller1.ButtonR2.pressing()){
    /*Clamp.setVelocity(200, percent);
    Clamp.spin(reverse);*/
    IntakeRoller.setVelocity(100, percent);
    IntakeRoller.spin(reverse);
  }
  else{
    /*Clamp.setStopping(hold);
    Clamp.stop();*/
    IntakeRoller.setStopping(hold);
    IntakeRoller.stop();
  }
}
/*double kP = 0.5;
double kD = 0.2;
void TwoMotorFly(double t){
 double c = ((Flywheel1.voltage() + Flywheel2.voltage()) / 2);
 double pe = 0;
 while (t-c > 0){
   double c = ((Flywheel1.voltage() + Flywheel2.voltage()) / 2);
   double e = t - c;
   double d = pe - e;
   pe = e;
   Flywheel1.spin(forward, e * kP + kD * d, volt);
   Flywheel2.spin(forward, e * kP + kD * d, volt);
 }
 Flywheel2.stop();
 Flywheel1.stop();
 
}*/

void flywheelMovement() {
  if(Controller1.ButtonY.pressing()){
    Flywheel1.setVelocity(79, percent);
    Flywheel2.setVelocity(79, percent);
    Flywheel1.spin(forward);
    Flywheel2.spin(reverse);
  }
  else {
    Flywheel1.setStopping(coast);
    Flywheel2.setStopping(coast);
    Flywheel1.stop();
    Flywheel2.stop();
  }
}

/*void flywheelMovementSlow() {
  if(Controller1.ButtonX.pressing()){
    Flywheel1.setVelocity(65, percent);
    Flywheel2.setVelocity(65, percent);
    Flywheel1.spin(forward);
    Flywheel2.spin(reverse);
  }
  else {
    Flywheel1.setStopping(coast);
    Flywheel2.setStopping(coast);
    Flywheel1.stop();
    Flywheel2.stop();
  }f
}*/

void indexerMovement() {
  if(Controller1.ButtonL2.pressing()) {
    Indexer.setVelocity(100, percent);
    Indexer.spin(forward);
  }
  else if (Controller1.ButtonL1.pressing()) {
    Indexer.setVelocity(100,percent);
    Indexer.spin(reverse);
  }
  else {
    Indexer.setStopping(hold);
    Indexer.stop();
  }
}

void turnCounterClockwise(double amount){
  Inertial.setRotation(0, degrees);
  while(fabs(Inertial.rotation(degrees)) < amount){
    double error = amount - fabs(Inertial.rotation(degrees));
    LeftBack.spin(reverse, 0.2*error + 5, percent);
    RightBack.spin(forward, 0.2*error + 5, percent);
    LeftFront.spin(reverse, 0.2*error + 5, percent);
    RightFront.spin(forward, 0.2*error + 5, percent);
    RightMiddle.spin(forward, 0.2*error + 5, percent);
    LeftMiddle.spin(forward, 0.2*error + 5, percent);
    wait(5, msec);
  }
  LeftBack.setStopping(hold);
  RightBack.setStopping(hold);
  RightFront.setStopping(hold);
  LeftFront.setStopping(hold);
  RightMiddle.setStopping(hold);
  LeftMiddle.setStopping(hold);
  LeftBack.stop();
  RightBack.stop();
  RightFront.stop();
  LeftFront.stop();
  RightMiddle.stop();
  LeftMiddle.stop();
  wait(0.5, sec);
}

void turnClockwise(double amount){
  Inertial.setRotation(0, degrees);
  while(fabs(Inertial.rotation(degrees))< amount){
    double error = amount - fabs(Inertial.rotation(degrees));
    LeftBack.spin(forward, 0.2*error + 5, percent);
    RightBack.spin(reverse, 0.2*error + 5, percent);
    LeftFront.spin(forward, 0.2*error + 5, percent);
    RightFront.spin(reverse, 0.2*error + 5, percent);
    RightMiddle.spin(reverse, 0.2*error + 5, percent);
    LeftMiddle.spin(reverse, 0.2*error + 5, percent);
    wait(5, msec);
  }
  LeftBack.setStopping(hold);
  RightBack.setStopping(hold);
  RightFront.setStopping(hold);
  LeftFront.setStopping(hold);
  LeftMiddle.setStopping(hold);
  RightMiddle.setStopping(hold);
  LeftBack.stop();
  RightBack.stop();
  RightFront.stop();
  LeftFront.stop();
  LeftMiddle.stop();
  RightMiddle.stop();
  wait(0.5, sec);
}
//BEN'S HELPER FUNCTIONS------------------------------------------------------------

void moveDrivetrain(float vel, int dist, bool smooth, bool sync) {
  LeftFront.setStopping(coast);
  LeftBack.setStopping(coast);
  RightFront.setStopping(coast);
  RightBack.setStopping(coast);
  RightMiddle.setStopping(coast);
  LeftMiddle.setStopping(coast); 
  LeftFront.setVelocity(vel, percent);
  LeftBack.setVelocity(vel, percent);
  RightFront.setVelocity(vel, percent);
  RightBack.setVelocity(vel, percent);
  RightMiddle.setVelocity(vel, percent);
  LeftMiddle.setVelocity(vel, percent);

  if (smooth) {
    LeftBack.setPosition(0, degrees);

    LeftFront.spin(forward);
    LeftBack.spin(forward);
    RightFront.spin(forward);
    RightBack.spin(forward);
    RightMiddle.spin(forward);
    LeftMiddle.spin(forward);

    while (std::abs(LeftBack.position(degrees)) < std::abs(dist)) {
      wait(10, msec);
    }

    LeftFront.stop();
    LeftBack.stop();
    RightFront.stop();
    RightBack.stop();
    RightMiddle.stop();
    LeftMiddle.stop();

  } else {
    LeftFront.spinFor(forward, dist, degrees, false);
    LeftBack.spinFor(forward, dist, degrees, false);
    RightFront.spinFor(forward, dist, degrees, false);
    RightBack.spinFor(forward, dist, degrees, false);
    RightMiddle.spinFor(forward, dist, degrees, false);
    LeftMiddle.spinFor(forward, dist, degrees, sync);
  }
}

//----------------------------------------------------------------------------------

int selected = 0;
std::string autons[8] = {"Disabled", "1 Roller", "1 Roller + Low Goal", "AWP Right", "2 Goal Right", "Right Neutral AWP", "Right Mid", "AWP2 from Left"};
int size = sizeof(autons);

bool elevated = false;

void autonSelector(){
  Controller1.Screen.clearScreen();
  task::sleep(100);
  while(true){
    Controller1.Screen.clearScreen();
    task::sleep(100);
    Controller1.Screen.clearLine(2);
    Controller1.Screen.setCursor(2,1);
    Controller1.Screen.print((autons[selected] + ",").c_str());
    Controller1.Screen.newLine();
    Controller1.Screen.print((elevated ? "Elevated" : "Default"));
    task::sleep(100);
     if(Controller1.ButtonRight.pressing()){
      elevated = !elevated;
        if (!elevated) {
          selected = (selected + 1 + size) % size;
        }
     }else if(Controller1.ButtonLeft.pressing()){
       elevated = !elevated;
       if (elevated) {
        selected = (selected - 1 + size) % size;
       }
     }else if(Controller1.ButtonA.pressing()){
       task::sleep(100);
       if(Controller1.ButtonA.pressing()){
         goto slctEnd;
       }
     }
   }
   slctEnd:
   Controller1.rumble("..");
}

void pre_auton(void) {
 // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Brain.Screen.drawImageFromFile("bike discord banner.png", 0, 0);
  RightLift.stop(hold);
  Inertial.calibrate();
  wait(3, sec);
  autonSelector();

}
/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*  This task is used to control the robot during the autonomous phase of    */
/*  a VEX Competition.                                                       */
/*---------------------------------------------------------------------------*/
void autonomous(void) {
  int x = 980; // Degrees for moving forward to the neutral goal
  switch(selected){
    case 0:{ //Disabled
      break;
    }
    case 1:{ //1 Roller
      IntakeRoller.setVelocity(100, percent);
      IntakeRoller.spinFor(forward, 5000, degrees, false);

      move(forward, 50);

      move(reverse, 75);
      break;
    }
    case 2: { //1 Roller + Low Goal
      IntakeRoller.setVelocity(100, percent);
      IntakeRoller.spinFor(forward, 5000, degrees, false);

      move(forward, 50);

      move(reverse, 75);

      turn(::left, 250);

      Indexer.setVelocity(100, pct);
      Flywheel1.setVelocity(69, pct);
      Flywheel2.setVelocity(69, pct);

      wait(2000, msec);

      Indexer.spinFor(forward, 300, degrees, true);
      break;
    }
    case 3: { //Right Neutral AWP
      setStopping(coast);
      setVelocity(100);
      LeftFront.setPosition(0, degrees);
      
      // setting up for auton
      RightLift.spinFor(reverse, 50, degrees, false);
      
      // spinning forward towards the goal
      
      move(forward, x);

      wait(100, msec);

      // clamp down
      
      ClampSolenoid.set(true);

      // moving backwards with the goal

      move(reverse, x);

      // turning 90°
      
      turn(::left, 250);

      // moving backwards to place ring in alliance goal
      
      move(reverse, 100);

      // placing ring in alliance goal
      
      Sporklift.spinFor(forward, 50, degrees, true);

      // setting up to pickup alliance goal by moving forward
      
      move(forward, 135);

      // 2nd part of setting up to pick up alliance goal; moving forklift down
      
      Sporklift.spinFor(forward, 50, degrees, true);
      
      // 3rd part of setting up; moving backwards with forklift down
      
      move(reverse, 150);

      // 4th part of getting AWP; forklifting up to pick up goal
      
      Sporklift.spinFor(reverse, 100, degrees, true);
      
      // moving forward to get the AWP
      
      move(forward, 145);
      
      break; 
    }
    case 4: { //Right Mid
      setStopping(coast);
      setVelocity(100);
      RightLift.spinFor(reverse, 50, degrees, false);
      if (elevated) {
        move(forward, x + 410);
      } else {
        move(forward, x + 430);
      }
      
      wait(100, msec);

      ClampSolenoid.set(true);
      
      move(reverse, x + 230);
      break;
    }
    case 5: { //AWP Carry from Left
      moveDrivetrain(100, 200, true, true);
      ClampSolenoid.set(true);
    }
  }
}

/*---------------------------------------------------------------------------*/
/*                                    Sporklift Code                         */
/*---------------------------------------------------------------------------*/

void sporkliftMovement() {
  if(Controller1.ButtonDown.pressing()){
    Sporklift.setVelocity(100, percent);
    Sporklift.spin(forward);
  }
  else if(Controller1.ButtonUp.pressing()){
    Sporklift.setVelocity(100, percent);
    Sporklift.spin(reverse);
  }
  else{
    Sporklift.setStopping(hold);
    Sporklift.stop();
  }
}

/*---------------------------------------------------------------------------*/
/*                              User Control Task                            */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*---------------------------------------------------------------------------*/
void usercontrol(void) {
 // User control code here, inside the loop
  while (1) {
    simpleDrive();
    armLift();
    intakeRollerMovement();
    flywheelMovement();
    //flywheelMovementSlow();
    indexerMovement();
    sporkliftMovement();
    platformMode();
    if(Controller1.ButtonLeft.pressing() && Controller1.ButtonRight.pressing()){
      RightLift.stop(hold);
      while((Controller1.ButtonY.pressing() && Controller1.ButtonA.pressing()) == false){
        goSlow();
        wait(10, msec);
      }
    }
    wait(15, msec);
  } // Sleep the task for a short amount of time to prevent wasted resources.
}

int main() {
 // Set up callbacks for autonomous and driver control periods.
 Competition.autonomous(autonomous);
 Competition.drivercontrol(usercontrol);
 pre_auton();
 // Prevent main from exiting with an infinite loop.
 while (true) {
   wait(100, msec);
 }
}
