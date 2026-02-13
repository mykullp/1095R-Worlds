#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors

controller Controller1 = controller(primary);

motor Left1 = motor(PORT15, ratio6_1, false);
motor Left2 = motor(PORT17, ratio6_1, true);
motor Left3 = motor(PORT19, ratio6_1, true);

motor Right1 = motor(PORT18, ratio6_1, true);
motor Right2 = motor(PORT16, ratio6_1, false);
motor Right3 = motor(PORT20, ratio6_1, false);

motor_group leftMotors = motor_group(Left1,Left2,Left3); 
motor_group rightMotors = motor_group(Right1,Right2,Right3);

motor Intake = motor(PORT1, ratio6_1, true);

motor Wall = motor(PORT14, ratio36_1,false);

inertial Inertial = inertial(PORT21);
optical Optical = optical(PORT2);

distance VDistance = distance(PORT12);
distance HDistance = distance(PORT11);

digital_out Clamp = digital_out(Brain.ThreeWirePort.E);
digital_out Arm = digital_out(Brain.ThreeWirePort.F);
digital_out ArmClamp = digital_out(Brain.ThreeWirePort.D);
digital_out Tipper = digital_out(Brain.ThreeWirePort.C);

rotation Rotation = rotation(PORT13, true);
line Right_Line = line(Brain.ThreeWirePort.G);
line Left_Line = line(Brain.ThreeWirePort.H);

// VEXcode generated functions


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}