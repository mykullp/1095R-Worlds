using namespace vex;
#include "usercontrol.h"
#include "gui.h"
extern brain Brain;

// VEXcode devices
extern controller Controller1;

extern motor Right1;
extern motor Right2;
extern motor Right3;
extern motor Left1;
extern motor Left2;
extern motor Left3;

extern motor Intake;


extern motor Wall;

extern motor_group leftMotors;
extern motor_group rightMotors;

extern rotation liftTracker;

extern inertial Inertial;
extern optical Optical;
extern optical Optical2;
extern optical Optical3;
extern distance VDistance;
extern distance HDistance;

extern digital_out Clamp;
extern digital_out Arm;
extern digital_out ArmClamp;
extern digital_out Tipper;

extern pot Pot;
extern rotation Rotation;

extern line Right_Line;
extern line Left_Line;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 *8 This should be called at the start of your int main function.
 **/
void  vexcodeInit( void );