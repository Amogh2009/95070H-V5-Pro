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
// Controller1          controller                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      ``````````````````````
// LeftFront            motor         1               
// RightFront           motor         13              
// LeftBack             motor         17              
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

double speedFactor = 3;

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


void botTurn(turntype direction, int rotation) {
  LeftFront.spinFor(direction ? reverse : forward, rotation, degrees, false);
  LeftBack.spinFor(direction ? reverse : forward, rotation, degrees, false);
  RightFront.spinFor(direction ? forward : reverse, rotation, degrees, false);
  RightBack.spinFor(direction ? forward : reverse, rotation, degrees, true);
}

void move(vex::directionType direction, int time) {
  /*LeftFront.spinFor(direction, rotation, degrees, false);
  LeftBack.spinFor(direction, rotation, degrees, false);
  RightFront.spinFor(direction, rotation, degrees, false);
  RightBack.spinFor(direction, rotation, degrees, true);*/
  LeftFront.spin(direction);
  RightFront.spin(direction);
  LeftBack.spin(direction);
  RightBack.spin(direction);
  wait(time, msec);
  LeftFront.stop();
  RightFront.stop();
  LeftBack.stop();
  RightBack.stop();
  wait(500, msec);
}

<<<<<<< Updated upstream
void moveLeftDrivetrain(vex::directionType direction, int rotation) {
  LeftFront.spinFor(direction, rotation, degrees, false);
  LeftBack.spinFor(direction, rotation, degrees, true);
}

void moveRightDrivetrain(vex::directionType direction, int rotation) {
  RightFront.spinFor(direction, rotation, degrees, false);
  RightBack.spinFor(direction, rotation, degrees, true);
}

void botTurn3Motor(turntype direction, int rotation) {
  LeftFront.spinFor(direction ? reverse : forward, rotation, degrees, false);
  LeftBack.spinFor(direction ? reverse : forward, rotation, degrees, false);
  RightFront.spinFor(direction ? forward : reverse, rotation, degrees, true);
}

void botTurn2Motor(turntype direction, int rotation) {
  LeftBack.spinFor(direction ? reverse : forward, rotation, degrees, false);
  RightFront.spinFor(direction ? reverse : forward, rotation, degrees, true);
}

=======
>>>>>>> Stashed changes
void platformMode() {
  if(Controller1.ButtonY.pressing()){
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

/*void goSlow(){
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
} */

void simpleDrive(){
  double forwardAmount = Controller1.Axis3.position();
  double turnAmount = Controller1.Axis4.position(); //Axis 4 for unified joystick
  RightFront.spin(forward, (forwardAmount-turnAmount) / speedFactor, percent);
  RightBack.spin(forward, (forwardAmount-turnAmount) / speedFactor, percent);
  //RightMiddle.spin(forward, (forwardAmount-turnAmount) / speedFactor, percent);
  LeftFront.spin(forward, (forwardAmount+turnAmount) / speedFactor, percent);
  LeftBack.spin(forward, (forwardAmount+turnAmount) / speedFactor, percent);
  //LeftMiddle.spin(forward, (forwardAmount+turnAmount) / speedFactor, percent);
}
<<<<<<< Updated upstream

bool Controller1XY = true;

double fly_kp = 0.25; // how fast it increases
double fly_ki = 0.3; // how much offshoot/range of fluctuation
double fly_kd = 0.00005; // how many fluctuations are there
double speed_margin = 0;
double speed_marg_pct = 2;
bool flyescvar = false;
double speed_volt = 0;

//flywheel spin

void flywheel_spin_fwd(double flywheel_target_speed_pct) {
  
  Flywheel1.setVelocity(flywheel_target_speed_pct, pct);
  Flywheel2.setVelocity(flywheel_target_speed_pct, pct);
  Flywheel1.spin(directionType::fwd);
  Flywheel2.spin(directionType::rev);
}

//flywheel spin PID code
void flywheel_spin_fwd_PID(double flywheel_target_speed_pct){
  //speed_volt = 0;
double averagevolt = 0;
double preverror = 0;
double errorsum = 0;
double error = 0;
double derivative = 0;
double flywheel_target_speed_volt = (flywheel_target_speed_pct/100)*12;
Controller1.Screen.setCursor(3, 9);
Controller1.Screen.print("    ");
Controller1.Screen.setCursor(1,1);
Controller1.Screen.print("         ");
wait(10,msec);
 
 while (flyescvar == false) {
    averagevolt = ((Flywheel1.voltage() - Flywheel2.voltage()) / 2);
    //averagevolt = ((flywheelMotorA.velocity(velocityUnits::pct) + flywheelMotorB.velocity(velocityUnits::pct) ) / 2);
    error = flywheel_target_speed_volt - averagevolt;
    derivative = preverror - error;
    errorsum += error;
    preverror = error;
    speed_margin = fabs((error/flywheel_target_speed_volt) * 100);
    speed_volt =  error * fly_kp + fly_ki * errorsum + fly_kd * derivative;
  
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("C:%2.1fM:%2.0f", averagevolt,speed_margin);
    wait(5, msec);
  
    if(speed_margin <= speed_marg_pct) {
      flyescvar = true;
    } else {
        //flywheelMotorA.spin(forward, speed_volt, velocityUnits::pct);
        //flywheelMotorB.spin(forward, speed_volt, velocityUnits::pct);
        Flywheel1.spin(forward, speed_volt, volt);
        Flywheel2.spin(reverse, speed_volt, volt);
    }
    wait(10, msec);
  }
 Controller1.Screen.setCursor(3,9);
 Controller1.Screen.print("DONE");
 wait(5,msec);
 
 // Maintain the speed
 //flywheelMotorA.spin(forward, speed_volt, velocityUnits::pct);
 //flywheelMotorB.spin(forward, speed_volt, velocityUnits::pct);
 //Flywheel1.spin(forward, speed_volt, volt);
 //Flywheel2.spin(reverse, speed_volt, volt);
}

double preverror = 0;
double errorsum = 0;
double error = 0;
double derivative = 0;
void flyPIDadjustment(double flywheel_target_speed_pct) {
  double averagevolt = 0;
  
  double flywheel_target_speed_volt = (flywheel_target_speed_pct/100)*12;
  //Controller1.Screen.setCursor(3, 9);
  //Controller1.Screen.print("    ");
  //Controller1.Screen.setCursor(1,1);
  //Controller1.Screen.print("         ");
  //wait(10,msec);

  averagevolt = ((Flywheel1.voltage() - Flywheel2.voltage()) / 2);
  error = flywheel_target_speed_volt - averagevolt;
  derivative = preverror - error;
  errorsum += error;
  preverror = error;
  speed_margin = fabs((error/flywheel_target_speed_volt) * 100);
  speed_volt =  error * fly_kp + fly_ki * errorsum + fly_kd * derivative;
  
  Flywheel1.spin(forward, speed_volt, volt);
  Flywheel2.spin(reverse, speed_volt, volt);
}

void expansionMovement(void) {
  if((Controller1.ButtonUp.pressing()) && (Controller1.ButtonDown.pressing()) && (Controller1.ButtonLeft.pressing()) && (Controller1.ButtonRight.pressing())) {
    Expansion.set(true);
  } else {
    Expansion.set(false);
=======
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
>>>>>>> Stashed changes
  }
}
void pistonIndexerMovement(void) {
  if(Controller1.ButtonL1.pressing()) {
    pneumaticsIndexer.set(false);
    wait(200, msec);
    pneumaticsIndexer.set(true);
    wait(200, msec);
  } else if (Controller1.ButtonL2.pressing()) {
    pneumaticsIndexer.set(false);
    //wait(200, msec);
    pneumaticsIndexer.set(true);
    wait(200, msec);
    pneumaticsIndexer.set(false);
    wait(1200, msec);
    pneumaticsIndexer.set(true);
    wait(200, msec);
    pneumaticsIndexer.set(false);
    wait(1500, msec);
    pneumaticsIndexer.set(true);
    wait(200, msec);
  } else {
    pneumaticsIndexer.set(false);
  }
}

void TempBattery () {
  wait(30000, msec);
  
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("Motor Temps (%)");
  Controller1.Screen.newLine();
  Controller1.Screen.print(LeftBack.temperature(percent));
  Controller1.Screen.print(",");
  Controller1.Screen.print(LeftFront.temperature(percent));
  Controller1.Screen.print(",");
  Controller1.Screen.print(RightBack.temperature(percent));
  Controller1.Screen.print(",");
  Controller1.Screen.print(RightFront.temperature(percent));
  Controller1.Screen.print(",");
  Controller1.Screen.newLine();
  Controller1.Screen.print("Battery: ");
  Controller1.Screen.print(vexBatteryCapacityGet());
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

void flywheelSpin(double velocity) {
  Flywheel1.setVelocity(velocity, pct);
  Flywheel2.setVelocity(velocity, pct);
  Flywheel1.spin(fwd);
  Flywheel2.spin(reverse);
}

bool flywheelStart = false;
int flywheelVelocity = 80;

void flywheelRun() {
  flywheelStart = !flywheelStart;
  if(flywheelStart){ // fast shooter
    Flywheel1.setVelocity(flywheelVelocity, percent);
    Flywheel2.setVelocity(flywheelVelocity, percent);
    Flywheel1.spin(forward);
    Flywheel2.spin(reverse);
  } else {
    Flywheel1.setStopping(coast);
    Flywheel2.setStopping(coast);
    Flywheel1.stop();
    Flywheel2.stop();
  }
}

void flywheelSlow() {
    flywheelVelocity = 65;
    flywheelRun();
}

void flywheelFast() {
    flywheelVelocity = 80;
    flywheelRun();
}

<<<<<<< Updated upstream
void flywheelPIDFast(){
  flywheelStart = !flywheelStart;
  if(flywheelStart) {
    flywheel_spin_fwd_PID(69);
  } else {
    Flywheel1.setStopping(coast);
    Flywheel2.setStopping(coast);
    Flywheel1.spin(forward, 0, volt);
    Flywheel2.spin(reverse, 0, volt);
    flyescvar = false;
    //Flywheel1.stop();
    //Flywheel2.stop();
  }
}

void flywheelPIDSlow() {
  flywheelStart = !flywheelStart;
  if(flywheelStart) {
    flywheel_spin_fwd_PID(57);
  } else {
    Flywheel1.setStopping(coast);
    Flywheel2.setStopping(coast);
    Flywheel1.spin(forward, 0, volt);
    Flywheel2.spin(reverse, 0, volt);
    flyescvar = false;
    //Flywheel1.stop();
    //Flywheel2.stop();
  }
}

void flywheelPIDmovement() {
  if(Controller1.ButtonX.pressing()) {
    flyPIDadjustment(90);
  }
  else if(Controller1.ButtonY.pressing()) {
    flyPIDadjustment(75);
  }
  else if(!Controller1.ButtonX.pressing() && !Controller1.ButtonY.pressing()) {
    Flywheel1.stop();
    Flywheel2.stop();
  }
}

=======
>>>>>>> Stashed changes
void flywheelMovement() {
    Controller1.ButtonY.pressed(flywheelFast);
    Controller1.ButtonX.pressed(flywheelSlow);
<<<<<<< Updated upstream
    */
    /*
    if (Controller1.ButtonY.pressing()) {
       flywheel_spin_fwd_PID(70);
       Controller1XY = false;
     } else if (Controller1.ButtonX.pressing()) {
       flywheel_spin_fwd_PID(35);
       Controller1XY = false;
     } else if (!Controller1XY) {
       flyescvar = true;
       speed_volt = 0;
       wait(20,msec);
       Flywheel1.stop();
       Flywheel2.stop();
       flyescvar = false;
     }
     */
     
    if(Controller1.ButtonY.pressing()){
      Flywheel1.setVelocity(81, pct);
      Flywheel2.setVelocity(81, pct);
      Flywheel1.spin(forward);
      Flywheel2.spin(reverse);
      //flyescvar = false;
      //flywheel_spin_fwd_PID(85);
      Controller1XY = false;
    } else if(Controller1.ButtonX.pressing()) {
      /*Flywheel1.setVelocity(77, pct);
      Flywheel2.setVelocity(77, pct);
      Flywheel1.spin(forward);
      Flywheel2.spin(reverse);*/
      Flywheel1.setVelocity(77, pct);
      Flywheel2.setVelocity(77, pct);
      Flywheel1.spin(fwd);
      Flywheel2.spin(reverse);
      Controller1XY = false;
    } else if(!Controller1XY) {
      Flywheel1.setStopping(coast);
      Flywheel2.setStopping(coast);
      Flywheel1.stop();
      Flywheel2.stop();
      Controller1XY = true;
      flyescvar = false;
    }
=======
>>>>>>> Stashed changes
}

void indexerMovement() {
  if(Controller1.ButtonL1.pressing()) {
    Indexer.setVelocity(100, percent);
    Indexer.spinFor(forward, 200, degrees, true);
    Indexer.spinFor(reverse, 200, degrees, true);
  }
  else if (Controller1.ButtonL2.pressing()) {
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
std::string autons[8] = {"Disabled", "1 Roller", "1 Roller + Low Goal", "Disc Shooter", "Roller Other Side", "Disc Shooter Two", "Skills Roller", "AWP2 from Left"};
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

void autonIndexer(void) {
  pneumaticsIndexer.set(true);
  wait(200, msec);
  pneumaticsIndexer.set(false);
  wait(200, msec);
}

void pre_auton(void) {
 // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  pneumaticsIndexer.set(false);
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
      IntakeRoller.spinFor(reverse, 5000, degrees, false);

      move(forward, 50);

      move(reverse, 75);
      break;
    }
    case 2: { //1 Roller + Low Goal
      IntakeRoller.setVelocity(100, percent);
      IntakeRoller.spinFor(reverse, 800, degrees, false);

<<<<<<< Updated upstream
      move(forward, 200);

      //move(reverse, 200);
=======
      move(forward, 50);

      move(reverse, 75);
>>>>>>> Stashed changes

      flywheelSpin(70);

      wait(2500, msec);

<<<<<<< Updated upstream
      LeftFront.spin(reverse);
      LeftBack.spin(reverse);
      RightFront.spin(reverse);
      RightBack.spin(reverse);
=======
      Indexer.setVelocity(5, pct);
      Indexer.spinFor(fwd, 300, deg, false);
>>>>>>> Stashed changes

      wait(300, msec);
      
      LeftFront.stop();
      LeftBack.stop();
      RightFront.stop();
      RightBack.stop();

      wait(1, sec);

      LeftFront.spin(reverse);
      LeftBack.spin(reverse);
      RightFront.spin(forward);
      RightBack.spin(forward);

      wait (850, msec);

      LeftFront.stop();
      LeftBack.stop();
      RightFront.stop();
      RightBack.stop();

      wait (500, msec);

      autonIndexer();
      wait(500, msec);
      autonIndexer();

      wait(2, sec);

      Flywheel1.stop();
      Flywheel2.stop();
      break;
    }
<<<<<<< Updated upstream
    case 3: { //Disc Shooter
      botTurn3Motor(::left, 35);
      flywheelSpin(91.2);
      wait(4200, msec);
      //flywheel_spin_fwd_PID(95);
      //wait(3000, msec);
      autonIndexer();
      wait(4000, msec);
      autonIndexer();
      wait(200, msec);
      Flywheel1.stop();
      Flywheel2.stop();
      wait(500, msec);
      LeftFront.spin(reverse);
      LeftBack.spin(reverse);
      RightFront.spin(forward);
      RightBack.spin(forward);
      wait(350, msec);
      LeftFront.stop();
      LeftBack.stop();
      RightFront.stop();
      RightBack.stop();
      //
      wait(500, msec);
      LeftFront.spin(forward);
      LeftBack.spin(forward);
      RightFront.spin(forward);
      RightBack.spin(forward);
      wait(250, msec);
      LeftFront.stop();
      LeftBack.stop();
      RightFront.stop();
      RightBack.stop();
      wait(500, msec);
      LeftFront.spin(reverse);
      LeftBack.spin(reverse);
      RightFront.spin(forward);
      RightBack.spin(forward);
      wait(1300, msec);
      LeftFront.stop();
      LeftBack.stop();
      RightFront.stop();
      RightBack.stop();
      wait(500, msec);
      IntakeRoller.spinFor(reverse, 800, degrees, false);
      LeftFront.spin(forward);
      LeftBack.spin(forward);
      RightFront.spin(forward);
      RightBack.spin(forward);
      wait(800, msec);
      LeftFront.stop();
      LeftBack.stop();
      RightFront.stop();
      RightBack.stop();
      break; 
    }
    case 4: { //Roller Other Side

      LeftFront.spin(forward);
      LeftBack.spin(forward);
      RightFront.spin(forward);
      RightBack.spin(forward);

      wait(1900, msec);

      LeftFront.stop();
      LeftBack.stop();
      RightFront.stop();
      RightBack.stop();

      wait(300, msec);

      LeftFront.spin(forward);
      LeftBack.spin(forward);
      RightFront.spin(reverse);
      RightBack.spin(reverse);

      wait(950, msec);

      LeftFront.stop();
      LeftBack.stop();
      RightFront.stop();
      RightBack.stop();

      wait(300, msec);

      IntakeRoller.spinFor(reverse, 400, degrees, false);

      LeftFront.spin(forward);
      LeftBack.spin(forward);
      RightFront.spin(forward);
      RightBack.spin(forward);

      wait(400, msec);

      LeftFront.stop();
      LeftBack.stop();
      RightFront.stop();
      RightBack.stop();
      break;
    }
    case 5: { //Disc Shooter 2
      IntakeRoller.setVelocity(100, percent);
      IntakeRoller.spinFor(reverse, 400, degrees, false);

      move(forward, 200);
      //move(reverse, 200);

      flywheelSpin(85);

      //wait(2500, msec);

      LeftFront.spin(reverse);
      LeftBack.spin(reverse);
      RightFront.spin(reverse);
      RightBack.spin(reverse);

      wait(250, msec);
      
      LeftFront.stop();
      LeftBack.stop();
      RightFront.stop();
      RightBack.stop();

      wait(500, msec);

      LeftFront.spin(reverse);
      LeftBack.spin(reverse);
      RightFront.spin(fwd);
      RightBack.spin(fwd);

      wait (1200, msec);

      LeftFront.stop();
      LeftBack.stop();
      RightFront.stop();
      RightBack.stop();

      wait (300, msec);

      IntakeRoller.spin(forward);

      move(forward, 1650);

      move(forward, 1200);

      //IntakeRoller.stop();

      LeftFront.spin(reverse);
      LeftBack.spin(reverse);
      RightFront.spin(fwd);
      RightBack.spin(fwd);

      wait (900, msec);

      LeftFront.stop();
      LeftBack.stop();
      RightFront.stop();
      RightBack.stop();

      wait (300, msec);
      move(fwd, 500);
      IntakeRoller.stop();

      autonIndexer();
      wait(2000, msec);
      autonIndexer();
      wait(2000, msec);
      autonIndexer();
      wait(1000, msec);
      autonIndexer();

      wait(2, sec);

      Flywheel1.stop();
      Flywheel2.stop();
=======
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
      
      botTurn(::left, 250);

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
    case 4: { //Turning Test
      botTurn(::left, 90);
      botTurn(::right, 90);
      break;
    }
    case 5: { //AWP Carry from Left
      moveDrivetrain(100, 200, true, true);
      ClampSolenoid.set(true);
>>>>>>> Stashed changes
      break;
    }
    case 6: { // Skills Roller
    setStopping(hold);
    setVelocity(69);
    IntakeRoller.spin(reverse);
    move(fwd, 200);
    wait(120, msec);
    IntakeRoller.stop();
    move(reverse, 630);
    LeftFront.spin(fwd);
    LeftBack.spin(fwd);
    RightFront.spin(reverse);
    RightBack.spin(reverse);
    wait(120, msec);
    move(fwd, 200);
    Expansion.set(true);
    break;
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
  flywheelMovement();
  while (1) {
    simpleDrive();
    armLift();
    //TempBattery();
    intakeRollerMovement();
<<<<<<< Updated upstream
    pistonIndexerMovement();
    expansionMovement();
    flywheelPIDmovement();
=======
>>>>>>> Stashed changes
    indexerMovement();
    sporkliftMovement();
    platformMode();
    /*if(Controller1.ButtonLeft.pressing() && Controller1.ButtonRight.pressing()){
      RightLift.stop(hold);
      while((Controller1.ButtonY.pressing() && Controller1.ButtonA.pressing()) == false){
        //goSlow();
        wait(10, msec);
      }
    }*/
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
<<<<<<< Updated upstream


=======
>>>>>>> Stashed changes
