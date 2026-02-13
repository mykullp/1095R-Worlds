#include "vex.h"
#include "gui_images/logo copy.h"

using namespace vex;
competition Competition;

Drive chassis(

//Specify your drive setup below. There are seven options:
//ZERO_TRACKER_NO_ODOM, ZERO_TRACKER_ODOM, TANK_ONE_ENCODER, TANK_ONE_ROTATION, TANK_TWO_ENCODER, TANK_TWO_ROTATION, HOLONOMIC_TWO_ENCODER, and HOLONOMIC_TWO_ROTATION
//For example, if you are not using odometry, put ZERO_TRACKER_NO_ODOM below:
TANK_TWO_ROTATION,

//Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
//You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

//Left Motors:
motor_group(Left1, Left2, Left3),

//Right Motors:
motor_group(Right1, Right2, Right3),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT7,

//Input your wheel diameter. (4" omnis are actually closer to 4.125"):
2.75,
//External ratio, must be in decimal, in the format of input teeth/output teeth.
//If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
//If the motor drives the wheel directly, this value is 1:
0.75,


//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
//For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
358,

/*---------------------------------------------------------------------------*/
/*                                  PAUSE!                                   */
/*                                                                           */
/*  The rest of the drive constructor is for robots using POSITION TRACKING. */
/*  If you are not using position tracking, leave the rest of the values as  */
/*  they are.                                                                */
/*---------------------------------------------------------------------------*/

//If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.

//FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
//LF:      //RF:    
PORT5,     -PORT2,

//LB:      //RB: 
PORT3,     -PORT4,

//If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
//If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
//If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
PORT8,

//Input the Forward Tracker diameter (reverse it to make the direction switch):
2.00,

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
//This distance is in inches:
0,

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
PORT9,

//Sideways tracker diameter (reverse to make the direction switch):
-2.00,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
0  //1.45

);

Lift arm(
  // ONE_ROTATION or NO_ROTATION for sensors
  ONE_ROTATION, 

  // Motor Ports here
  motor_group(Wall),

  // Port of Rotation Sensor, if no Rotation Sensor, choose a motor port
  PORT13,

  // Gear Ratio from Motor to Lift (A 12 to 48 tooth would be 0.25)
  0.333
);

int current_auton_selection = 1; //THIS DOES NOTHING? - JOSEPH 10/27


void pre_auton(void) {
  Inertial.startCalibration(3000);
  Inertial.calibrate(3000);
  vexcodeInit();
  vex::task GUI(*gui);
  default_constants();
   
  Clamp.set(false);
  Arm.set(false);
  ArmClamp.set(false);

  Optical.setLightPower(100, percent);

  Optical.integrationTime(5);

  Intake.setVelocity(100,pct);

  arm.LiftMotors.setStopping(brake);
  arm.lift_pid = true;
  arm.max_voltage = 6;
  arm.lift_task.stop();
  arm.start_lift(state[0]);

  auto_started = false;
}

void score_alliance(){
  chassis.drive_settle_time = 50;
  chassis.drive_timeout = 800;
  arm.max_voltage = 12;
  arm.lift_pid = true;
  lift_move = true;
  Intake.spinToPosition(Intake.position(degrees) - 100, degrees, false);
  arm.replace_target(alliance_pos);
  chassis.drive_distance(-6.5, chassis.get_absolute_heading(),6.5,0);
  chassis.DriveL.setStopping(coast);
  chassis.DriveR.setStopping(coast);
  lift_move = false;
}

void autonomous(void) {
  run_auto();

  //tester();

}

void usercontrol(void) {
  Clamp.set(clampDown);
  Arm.set(armDown);
  ArmClamp.set(clawDown);
  Tipper.set(tipper_out);

  leftMotors.setStopping(coast);
  rightMotors.setStopping(coast);

  runTimer = false;
  auto_started = true;
  colorsort_on = true;
  antijam_on = true;
  chassis.set_coordinates(-40,45,90);

  leftMotors.setStopping(coast);
  rightMotors.setStopping(coast);
  
  vex::task driver_cont(user_cont);
  vex::task lift_cont(lift_macro);
  vex::task pneumatics(pneumatics_toggle);

  printf("%d\n",TeamColor);
  while (1) { 
    if(!climb_toggle){
      chassis.single_arcade();
    } else {
      chassis.capped_drive(5);
    }
    if(Controller1.ButtonDown.pressing()){
      score_alliance();
      waitUntil(!Controller1.ButtonDown.pressing());
    }
    task::sleep(5);
  } 
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  while (true) {
    wait(100, msec);
  }
}