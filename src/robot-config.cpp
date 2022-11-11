#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor LeftFront = motor(PORT4, ratio18_1, true);
motor RightFront = motor(PORT13, ratio18_1, false);
motor LeftBack = motor(PORT17, ratio18_1, true);
motor RightBack = motor(PORT11, ratio18_1, false);
motor RightLift = motor(PORT5, ratio18_1, false);
inertial Inertial = inertial(PORT21);
digital_out Expansion = digital_out(Brain.ThreeWirePort.D);
motor Sporklift = motor(PORT19, ratio18_1, false);
motor RightMiddle = motor(PORT14, ratio18_1, false);
digital_out ClampSolenoid = digital_out(Brain.ThreeWirePort.A);
motor IntakeRoller = motor(PORT20, ratio18_1, false);
motor Flywheel1 = motor(PORT16, ratio18_1, false);
motor LeftMiddle = motor(PORT6, ratio18_1, false);
motor Flywheel2 = motor(PORT9, ratio18_1, false);
motor Indexer = motor(PORT10, ratio18_1, false);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}