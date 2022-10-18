using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor LeftFront;
extern motor RightFront;
extern motor LeftBack;
extern motor RightBack;
extern motor Indexer;
extern motor IntakeRoller;
extern motor RightLift;
extern inertial Inertial;
extern digital_out OldbackPiston;
extern motor Sporklift;
extern motor RightMiddle;
extern digital_out ClampSolenoid;
extern motor Flywheel1;
extern motor LeftMiddle;
extern motor Flywheel2;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );