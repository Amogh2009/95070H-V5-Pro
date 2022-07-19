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
// FrontLeft            motor         1               
// FrontRight           motor         2               
// BackLeft             motor         3               
// BackRight            motor         4               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>
#include <string>

using namespace vex;
// A global instance of competition
competition Competition;
//Function for determining whether input is positive, negative, or 0

int speedFactor = 1;

void simpleDrive() {
  double forwardAmount = Controller1.Axis3.position();
  double turnAmount = Controller1.Axis1.position();

  FrontRight.spin(forward, (forwardAmount-turnAmount) / speedFactor, percent);
  FrontLeft.spin(forward, (forwardAmount-turnAmount) / speedFactor, percent);
  BackRight.spin(forward, (forwardAmount+turnAmount) / speedFactor, percent);
  BackLeft.spin(forward, (forwardAmount+turnAmount) / speedFactor, percent);
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
}