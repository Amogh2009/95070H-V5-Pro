using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor LeftFront;
extern motor RightFront;
extern motor LeftBack;
extern motor RightBack;
extern inertial Inertial;
extern digital_out Expansion;
extern digital_out pneumaticsIndexer;
extern motor IntakeRoller;
extern motor Flywheel1;
extern motor Flywheel2;
extern motor Indexer;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );