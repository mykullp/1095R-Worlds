#include "vex.h"
#include "particle_filter/odom.cpp"

Drive::Drive(enum::drive_setup drive_setup, motor_group DriveL, motor_group DriveR, int gyro_port, float wheel_diameter, float wheel_ratio, float gyro_scale, int DriveLF_port, int DriveRF_port, int DriveLB_port, int DriveRB_port, int ForwardTracker_port, float ForwardTracker_diameter, float ForwardTracker_center_distance, int SidewaysTracker_port, float SidewaysTracker_diameter, float SidewaysTracker_center_distance) :
  wheel_diameter(wheel_diameter),
  wheel_ratio(wheel_ratio),
  gyro_scale(gyro_scale),
  drive_in_to_deg_ratio(wheel_ratio/360.0*M_PI*wheel_diameter),
  ForwardTracker_center_distance(ForwardTracker_center_distance),
  ForwardTracker_diameter(ForwardTracker_diameter),
  ForwardTracker_in_to_deg_ratio(M_PI*ForwardTracker_diameter/360.0),
  SidewaysTracker_center_distance(SidewaysTracker_center_distance),
  SidewaysTracker_diameter(SidewaysTracker_diameter),
  SidewaysTracker_in_to_deg_ratio(M_PI*SidewaysTracker_diameter/360.0),
  drive_setup(drive_setup),
  DriveL(DriveL),
  DriveR(DriveR),
  Gyro(inertial(gyro_port)),
  DriveLF(DriveLF_port, is_reversed(DriveLF_port)),
  DriveRF(DriveRF_port, is_reversed(DriveRF_port)),
  DriveLB(DriveLB_port, is_reversed(DriveLB_port)),
  DriveRB(DriveRB_port, is_reversed(DriveRB_port)),
  R_ForwardTracker(ForwardTracker_port),
  R_SidewaysTracker(SidewaysTracker_port),
  E_ForwardTracker(ThreeWire.Port[to_port(ForwardTracker_port)]),
  E_SidewaysTracker(ThreeWire.Port[to_port(SidewaysTracker_port)])
{
  if (drive_setup != ZERO_TRACKER_NO_ODOM){
    if (drive_setup == TANK_ONE_ENCODER || drive_setup == TANK_ONE_ROTATION || drive_setup == ZERO_TRACKER_ODOM){
      odom.set_physical_distances(ForwardTracker_center_distance, 0);
    } else {
      odom.set_physical_distances(ForwardTracker_center_distance, SidewaysTracker_center_distance);
    }
  }
}

void Drive::set_sideways(float distance){
  chassis.SidewaysTracker_center_distance = distance;
}
void Drive::drive_with_voltage(float leftVoltage, float rightVoltage){
  DriveL.spin(fwd, leftVoltage, volt);
  DriveR.spin(fwd, rightVoltage,volt);
}

void Drive::drive_with_velocity(float leftVelocity, float rightVelocity){
  DriveL.spin(fwd, leftVelocity, pct);
  DriveR.spin(fwd, rightVelocity, pct);
}

void Drive::set_turn_constants(float turn_max_voltage, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  this->turn_max_voltage = turn_max_voltage;
  this->turn_kp = turn_kp;
  this->turn_ki = turn_ki;
  this->turn_kd = turn_kd;
  this->turn_starti = turn_starti;
} 

void Drive::set_drive_constants(float drive_max_voltage, float drive_kp, float drive_ki, float drive_kd, float drive_starti){
  this->drive_max_voltage = drive_max_voltage;
  this->drive_kp = drive_kp;
  this->drive_ki = drive_ki;
  this->drive_kd = drive_kd;
  this->drive_starti = drive_starti;
} 

void Drive::set_heading_constants(float heading_max_voltage, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  this->heading_max_voltage = heading_max_voltage;
  this->heading_kp = heading_kp;
  this->heading_ki = heading_ki;
  this->heading_kd = heading_kd;
  this->heading_starti = heading_starti;
}

void Drive::set_swing_constants(float swing_max_voltage, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  this->swing_max_voltage = swing_max_voltage;
  this->swing_kp = swing_kp;
  this->swing_ki = swing_ki;
  this->swing_kd = swing_kd;
  this->swing_starti = swing_starti;
} 

void Drive::set_turn_exit_conditions(float turn_settle_error, float turn_settle_time, float turn_timeout){
  this->turn_settle_error = turn_settle_error;
  this->turn_settle_time = turn_settle_time;
  this->turn_timeout = turn_timeout;
}

void Drive::set_drive_exit_conditions(float drive_settle_error, float drive_settle_time, float drive_timeout){
  this->drive_settle_error = drive_settle_error;
  this->drive_settle_time = drive_settle_time;
  this->drive_timeout = drive_timeout;
}

void Drive::set_swing_exit_conditions(float swing_settle_error, float swing_settle_time, float swing_timeout){
  this->swing_settle_error = swing_settle_error;
  this->swing_settle_time = swing_settle_time;
  this->swing_timeout = swing_timeout;
}

float Drive::get_absolute_heading(){ 
  return( reduce_0_to_360( -Gyro.rotation()*360.0/gyro_scale ) ); 
}

float Drive::get_left_position_in(){
  return( DriveL.position(deg)*drive_in_to_deg_ratio );
}

float Drive::get_right_position_in(){
  return( DriveR.position(deg)*drive_in_to_deg_ratio );
}

double average(const std::vector<double>& vec) {
  double sum = std::accumulate(vec.begin(), vec.end(), 0.0); // Sum all elements
  return sum / vec.size(); // Divide by the number of elements
}

void Drive::turn_to_angle(float angle){
  turn_to_angle(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(float angle, float turn_max_voltage){
  turn_to_angle(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(float angle, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout){
  turn_to_angle(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(float angle, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  desired_heading = angle;
  PID turnPID(reduce_negative_180_to_180(angle - get_absolute_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
  while(turnPID.is_settled() == false){
    float error = -reduce_negative_180_to_180(angle - get_absolute_heading());
    float output = turnPID.compute(error);

    //printf("angle:%f\n", get_absolute_heading());
    output = clamp(output, -turn_max_voltage, turn_max_voltage);

    drive_with_voltage(output, -output);
    task::sleep(10);
  }
  DriveL.stop(hold);
  DriveR.stop(hold);
}

void Drive::dist_from_wall(double distance, float heading){
  drive_settled = false;
  PID drivePID(distance, drive_kp, drive_ki, drive_kd, drive_starti, 0.6, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(desired_heading - get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  /*while(drivePID.is_settled() == false){

    float drive_error = distance - Distance.objectDistance(inches);
    float heading_error = -reduce_negative_180_to_180(desired_heading - get_absolute_heading());
    float drive_output = drivePID.compute(drive_error);
    float heading_output = headingPID.compute(heading_error);

    //drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    drive_output = clamp_min_voltage(drive_output, drive_min_voltage);

    printf("heading: %f\n", heading_output);
    printf("drive_error:%f",drive_error);
    //printf("target: %f \ntravelled: %f \noutput: %f\n\n", distance, dist_travelled, drive_output);
    //printf("\n");
    if(Distance.isObjectDetected() == false){
      drive_output = -(drive_max_voltage/3);
    }

    if(drive_output < 0){
      drive_output = 0;
    }

    drive_with_voltage(drive_output+heading_output, drive_output-heading_output);
    task::sleep(10);
  }*/
  bool backwards = false;
  if(distance - VDistance.objectDistance(inches) < 0){
    drive_max_voltage *= -1;
    backwards = true;
  }
  drive_with_voltage(drive_max_voltage, drive_max_voltage);
  if (backwards){
    waitUntil(VDistance.objectDistance(inches) < distance);
  } else {
    waitUntil(VDistance.objectDistance(inches) > distance);
  }
  dist_travelled = 0;
  drive_settled = true;
  DriveL.stop(hold);
  DriveR.stop(hold);
}

void Drive::drive_to_roll(double angle, float heading, float roll_max_voltage, float roll_settle_error, float roll_settle_time, float roll_timeout, float roll_kp, float roll_ki, float roll_kd, float roll_starti){
  PID rollPID((angle - Inertial.roll()), roll_kp, roll_ki, roll_kd, roll_starti, roll_settle_error, roll_settle_time, roll_timeout);
  PID headingPID(reduce_negative_180_to_180(desired_heading - get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  while(rollPID.is_settled() == false){
    float error = -(angle-Inertial.roll());
    float output = rollPID.compute(error);

    float heading_error = -reduce_negative_180_to_180(desired_heading - get_absolute_heading());
    float heading_output = headingPID.compute(heading_error);

    output = clamp(output, -roll_max_voltage, roll_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_with_voltage(output+heading_output, output-heading_output);
    task::sleep(10);
  }
  DriveL.stop(hold);
  DriveR.stop(hold);
}

void Drive::drive_distance(float distance){
  drive_distance(distance, desired_heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading){
  drive_distance(distance, heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage){
  drive_distance(distance, heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  drive_distance(distance, heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  dist_travelled = 0;
  desired_heading = heading;
  drive_settled = false;
  PID drivePID(distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(desired_heading - get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
  float average_position = start_average_position;
  while(drivePID.is_settled() == false){
    average_position = (get_left_position_in()+get_right_position_in())/2.0;
    float drive_error = distance+start_average_position-average_position;
    float heading_error = -reduce_negative_180_to_180(chassis.desired_heading - get_absolute_heading());
    float drive_output = drivePID.compute(drive_error);
    float heading_output = headingPID.compute(heading_error);

    dist_travelled = average_position - start_average_position;

    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    drive_output = clamp_min_voltage(drive_output, drive_min_voltage);

    drive_with_voltage(drive_output+heading_output, drive_output-heading_output);
    task::sleep(10);
  }
  dist_travelled = 0;
  drive_settled = true;
  if(drive_min_voltage == 0){
    DriveL.stop(hold);
    DriveR.stop(hold);
  }
}

void Drive::left_swing_to_angle(float angle){
  left_swing_to_angle(angle, swing_max_voltage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti);
}

void Drive::left_swing_to_angle(float angle, float swing_max_voltage, float swing_settle_error, float swing_settle_time, float swing_timeout, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  desired_heading = angle;
  PID swingPID(reduce_negative_180_to_180(angle - get_absolute_heading()), swing_kp, swing_ki, swing_kd, swing_starti, swing_settle_error, swing_settle_time, swing_timeout);
  while(swingPID.is_settled() == false){
    float error = -reduce_negative_180_to_180(angle - get_absolute_heading());
    float output = swingPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    DriveL.spin(fwd, output, volt);
    DriveR.stop(hold);
    task::sleep(10);
  }
  DriveL.stop(hold);
  DriveR.stop(hold);
}

void Drive::right_swing_to_angle(float angle){
  right_swing_to_angle(angle, swing_max_voltage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti);
}

void Drive::right_swing_to_angle(float angle, float swing_max_voltage, float swing_settle_error, float swing_settle_time, float swing_timeout, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  desired_heading = angle;
  PID swingPID(reduce_negative_180_to_180(angle - get_absolute_heading()), swing_kp, swing_ki, swing_kd, swing_starti, swing_settle_error, swing_settle_time, swing_timeout);
  while(swingPID.is_settled() == false){
    float error = -reduce_negative_180_to_180(angle - get_absolute_heading());
    float output = swingPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    DriveR.spin(reverse, output, volt);
    DriveL.stop(hold);
    task::sleep(10);
  }
  DriveL.stop(hold);
  DriveR.stop(hold);
}

float Drive::get_ForwardTracker_position(){
  if (drive_setup==ZERO_TRACKER_ODOM){
    return(get_right_position_in());
  }
  if (drive_setup==TANK_ONE_ENCODER || drive_setup == TANK_TWO_ENCODER || drive_setup == HOLONOMIC_TWO_ENCODER){
    return(E_ForwardTracker.position(deg)*ForwardTracker_in_to_deg_ratio);
  }else{
    return(R_ForwardTracker.position(deg)*ForwardTracker_in_to_deg_ratio);
  }
}

float Drive::get_SidewaysTracker_position(){
  if (drive_setup==TANK_ONE_ENCODER || drive_setup == TANK_ONE_ROTATION || drive_setup == ZERO_TRACKER_ODOM){
    return(0);
  }else if (drive_setup == TANK_TWO_ENCODER || drive_setup == HOLONOMIC_TWO_ENCODER){
    return(E_SidewaysTracker.position(deg)*SidewaysTracker_in_to_deg_ratio);
  }else{
    return(R_SidewaysTracker.position(deg)*SidewaysTracker_in_to_deg_ratio);
  }
}

void Drive::position_track(){
  while(1){
    odom.update_position(get_ForwardTracker_position(), get_SidewaysTracker_position(), get_absolute_heading());
    //printf("X: %f\nY: %f\nAngle:%f\n\n", chassis.get_X_position(), chassis.get_Y_position(),chassis.get_absolute_heading());
    task::sleep(5);
  }
}

void Drive::set_heading(float orientation_deg){
  Gyro.setRotation(-orientation_deg*gyro_scale/360.0, deg);
  desired_heading = orientation_deg;
}

void Drive::set_coordinates(float X_position, float Y_position, float orientation_deg){
  odom.set_position(X_position, Y_position, orientation_deg, get_ForwardTracker_position(), get_SidewaysTracker_position());
  set_heading(orientation_deg);
  odom_task = task(position_track_task);
}

float Drive::get_X_position(){
  return(odom.X_position);
}

float Drive::get_Y_position(){
  return(odom.Y_position);
}

void switch_var(float a, float b) {
  a = a + b;  // Add both values and store in a
  b = a - b;  // Subtract the new a from b to get the original value of a
  a = a - b;  // Subtract the new b from a to get the original value of b
}

// Define wall constraints
float WALL_MAX = 71.0;  // Maximum distance to the wall (top-right corner)
float WALL_MIN = -71.0; // Minimum distance to the wall (bottom-left corner)

// Function to calculate the robot's X and Y coordinates based on the sensor data and angle
void reset_coordinates(bool V, bool H){
  // Get the sensor readings and position
  float x,y;
  float verticalDistance = vertical_dist();
  float horizontalDistance = horizontal_dist();
  float angle = chassis.get_absolute_heading();
  float currentX = chassis.get_X_position();
  float currentY = chassis.get_Y_position();

  // Calculate the robot's X and Y distances from the walls
  x = WALL_MAX - horizontalDistance;  
  y = WALL_MAX - verticalDistance;
  
  if(!VDistance.isObjectDetected() || !V){
    y = fabs(currentY);
  } 
  if(!HDistance.isObjectDetected() || !H){
    x = fabs(currentX);
  } 

  if(fabs(angle - 0) < 3 || fabs(angle - 180) < 3 || fabs(angle - 360) < 3){ // SWITCH SENSORS @ DIFF ANGLE
    y = WALL_MAX - horizontalDistance;  
    x = WALL_MAX - verticalDistance;
    if(!VDistance.isObjectDetected() || !V){
      x = fabs(currentX);
    } 
    if(!HDistance.isObjectDetected() || !H){
      y = fabs(currentY);
    } 
  } 
  
  if (currentX < 0 && currentY >= 0) {  // Quadrant II (Top-left)
    x *= -1;
    printf("quadrant 2\n");
  } else if (currentX < 0 && currentY < 0) {  // Quadrant III (Bottom-left)
    x *= -1;
    y *= -1;
    printf("quadrant 3\n");
  } else if (currentX >= 0 && currentY < 0) {  // Quadrant IV (Bottom-right)
    y *= -1;
    printf("quadrant 4\n");
  }
  if(fabs(hypot(x-currentX,y-currentY)) < 40){
    chassis.odom_task.stop();
    chassis.set_coordinates(x,y,chassis.get_absolute_heading());
    task::sleep(10);
    printf("RESET\nX: %f\nY: %f\nAngle:%f\n\n", chassis.get_X_position(), chassis.get_Y_position(),chassis.get_absolute_heading());
  } else {
    printf("X:%f\nY:%f\nToo risky to reset!", currentX, currentY);
  }
}

void Drive::drive_to_point(float X_position, float Y_position, bool forwards){
  drive_to_point(X_position, Y_position, forwards, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, bool forwards, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage){
  drive_to_point(X_position, Y_position, forwards, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, bool forwards, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  drive_to_point(X_position, Y_position, forwards, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, bool forwards, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  PID drivePID(hypot(X_position-get_X_position(),Y_position-get_Y_position()), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  float start_angle_deg = to_deg(atan2(Y_position-get_Y_position(),X_position-get_X_position()));
  PID headingPID(start_angle_deg-get_absolute_heading(), heading_kp, heading_ki, heading_kd, heading_starti);
  bool line_settled = is_line_settled(X_position, Y_position, start_angle_deg, get_X_position(), get_Y_position());;
  bool prev_line_settled = is_line_settled(X_position, Y_position, start_angle_deg, get_X_position(), get_Y_position());
  while(!drivePID.is_settled()){
    line_settled = is_line_settled(X_position, Y_position, start_angle_deg, get_X_position(), get_Y_position());
    if((!(line_settled == prev_line_settled))){ 
      break; 
      }
    prev_line_settled = line_settled;
    
    float robot_heading = forwards ? get_absolute_heading() : reduce_0_to_360(get_absolute_heading() + 180);
    float drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position());
    float heading_error = -reduce_negative_180_to_180(to_deg(atan2(Y_position-get_Y_position(),X_position-get_X_position())) - robot_heading);
    float drive_output = drivePID.compute(drive_error);

    if(!forwards){
      drive_output *= -1;
    }

    float heading_scale_factor = cos(to_rad(heading_error));
    drive_output*=heading_scale_factor;
    heading_error = reduce_negative_90_to_90(heading_error);
    float heading_output = headingPID.compute(heading_error);
    
    
    if (fabs(drive_error)<5) { heading_output = 0; }

    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_output = clamp_min_voltage(drive_output, drive_min_voltage);
    
    drive_with_voltage(drive_output+heading_output, drive_output-heading_output);
    task::sleep(10);
  }
  if(drive_min_voltage == 0){
    DriveL.stop(brake);
    DriveR.stop(brake);
  }
}


void Drive::straightline_to_point(float X_position, float Y_position, bool forward){
  straightline_to_point(X_position, Y_position, forward, 0, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::straightline_to_point(float X_position, float Y_position, bool forward, double extra_dist, float drive_max_voltage, float heading_max_voltage){
  straightline_to_point(X_position, Y_position, forward, extra_dist, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::straightline_to_point(float X_position, float Y_position, bool forward, double extra_dist, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  straightline_to_point(X_position, Y_position, forward, extra_dist, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::straightline_to_point(float X_position, float Y_position, bool forward, double extra_dist, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  desired_heading = forward ? to_deg(atan2(Y_position-get_Y_position(),X_position-get_X_position())) : reduce_negative_180_to_180((to_deg(atan2(Y_position-get_Y_position(),X_position-get_X_position())))+180);
  float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
  float target = forward ? hypot(X_position-get_X_position(),Y_position-get_Y_position()) + extra_dist : -hypot(X_position-get_X_position(),Y_position-get_Y_position()) - extra_dist;


  PID drivePID(target, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(desired_heading - get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);

  while(drivePID.is_settled() == false){
    

    float drive_error = target + start_average_position - ((get_left_position_in()+get_right_position_in())/2.0);
    float heading_error = reduce_negative_180_to_180(desired_heading - get_absolute_heading());
    float drive_output = drivePID.compute(drive_error);
  
    heading_error = reduce_negative_90_to_90(heading_error);
    float heading_output = headingPID.compute(heading_error);
    
    if ((fabs(drive_error))<(2*drive_settle_error)){ 
      heading_output = 0; 
    }
    
    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);
    drive_output = clamp_min_voltage(drive_output, drive_min_voltage);

    drive_with_voltage(drive_output-heading_output,drive_output+heading_output);
    task::sleep(10);
  }
  if(drive_min_voltage == 0){
    DriveL.stop(hold);
    DriveR.stop(hold);
  }
  desired_heading = get_absolute_heading();
}

//change atan
void Drive::turn_to_point(float X_position, float Y_position){
  turn_to_point(X_position, Y_position, 0, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg){
  turn_to_point(X_position, Y_position, extra_angle_deg, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout){
  turn_to_point(X_position, Y_position, extra_angle_deg, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  PID turnPID(reduce_negative_180_to_180(to_deg(atan2(Y_position-get_Y_position(),X_position-get_X_position())) - get_absolute_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
  while(turnPID.is_settled() == false){
    float error = reduce_negative_180_to_180(to_deg(atan2(Y_position-get_Y_position(),X_position-get_X_position())) - get_absolute_heading() + extra_angle_deg);
    float output = turnPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    drive_with_voltage(-output, output);
    task::sleep(10);

    
  }
  desired_heading = get_absolute_heading();
  DriveL.stop(hold);
  DriveR.stop(hold);
}

void Drive::drive_to_pose(float X_position, float Y_position, float angle, bool forward){
  drive_to_pose(X_position, Y_position, angle, forward, boomerang_lead, boomerang_setback, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_pose(float X_position, float Y_position, float angle, bool forward, float lead, float setback, float drive_min_voltage){
  drive_to_pose(X_position, Y_position, angle, forward, lead, setback, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_pose(float X_position, float Y_position, float angle, bool forward, float lead, float setback, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage){
  drive_to_pose(X_position, Y_position, angle, forward, lead, setback, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_pose(float X_position, float Y_position, float angle, bool forward, float lead, float setback, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  drive_to_pose(X_position, Y_position, angle, forward, lead, setback, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_pose(float X_position, float Y_position, float angle, bool forward, float lead, float setback, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  float target_distance = hypot(X_position-get_X_position(),Y_position-get_Y_position());
  PID drivePID(target_distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(to_deg(atan2(Y_position-get_Y_position(),X_position-get_X_position()))-get_absolute_heading(), heading_kp, heading_ki, heading_kd, heading_starti);
  bool line_settled = is_line_settled(X_position, Y_position, angle, get_X_position(), get_Y_position());
  bool prev_line_settled = is_line_settled(X_position, Y_position, angle, get_X_position(), get_Y_position());
  bool crossed_center_line = false;
  bool center_line_side = is_line_settled(X_position, Y_position, angle+90, get_X_position(), get_Y_position());
  bool prev_center_line_side = center_line_side;

  dist_travelled = 0;
  float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
  float average_position = start_average_position;

  bool cross = false;
  while(!drivePID.is_settled()){
    average_position = (get_left_position_in()+get_right_position_in())/2.0;
    dist_travelled = average_position - start_average_position;

    drivePID.error = hypot(X_position-get_X_position(),Y_position-get_Y_position());
    headingPID.error = to_deg(atan2(Y_position-get_Y_position(),X_position-get_X_position()))-get_absolute_heading();

    line_settled = is_line_settled(X_position, Y_position, angle, get_X_position(), get_Y_position());
    if(line_settled && !prev_line_settled){ break; }
    prev_line_settled = line_settled;

    center_line_side = is_line_settled(X_position, Y_position, angle+90, get_X_position(), get_Y_position());
    if(center_line_side != prev_center_line_side){
      crossed_center_line = true;
    }

    target_distance = hypot(X_position-get_X_position(),Y_position-get_Y_position());

    float carrot_X = forward ? X_position - cos(to_rad(angle)) * (lead * target_distance + setback) : X_position - cos(to_rad(angle+180)) * (lead * target_distance + setback);
    float carrot_Y = forward ? Y_position - sin(to_rad(angle)) * (lead * target_distance + setback) : Y_position - sin(to_rad(angle+180)) * (lead * target_distance + setback);

    float drive_error = hypot(carrot_X-get_X_position(),carrot_Y-get_Y_position());
    float heading_error = reduce_negative_180_to_180(to_deg(atan2(carrot_Y-get_Y_position(),carrot_X-get_X_position()))-get_absolute_heading());
  
    if (drive_error<drive_settle_error || crossed_center_line || drive_error < setback) { 
      cross = true;
    }
    
    if(cross){
      heading_error = reduce_negative_180_to_180(angle-get_absolute_heading()); 
      drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position());
    }
    
    if (!forward){
      if(!cross){
        heading_error = reduce_negative_180_to_180(to_deg(atan2(carrot_Y-get_Y_position(),carrot_X-get_X_position()))-get_absolute_heading() +180);
      }
    }

    float drive_output = drivePID.compute(drive_error);
    float heading_scale_factor;
    if(!cross){
      heading_scale_factor = cos(to_rad(heading_error));
    } else {
      heading_scale_factor = cos(to_rad(reduce_negative_180_to_180(to_deg(atan2(Y_position-get_Y_position(),X_position-get_X_position())))-get_absolute_heading()));
      if(!forward){
        heading_scale_factor*=-1;
      }
    }
    drive_output*=heading_scale_factor;
    heading_error = reduce_negative_90_to_90(heading_error);
    float heading_output = -headingPID.compute(heading_error);



    drive_output = clamp(drive_output, -fabs(heading_scale_factor)*drive_max_voltage, fabs(heading_scale_factor)*drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);
    drive_output = clamp_min_voltage(drive_output, drive_min_voltage);

    if(!forward){
      drive_output *=-1;
    }

    drive_with_voltage(left_voltage_scaling(drive_output, heading_output), right_voltage_scaling(drive_output, heading_output));
    task::sleep(10);
  }
  desired_heading = angle;
  if (drive_min_voltage == 0){
    DriveL.stop(hold);
    DriveR.stop(hold);
  }
}

std::tuple<int, Pose, float>crosstrack_error(Pose pose, std::vector<Pose> path, int last_index, int look_ahead){
  if (path.size() < 2){
    printf("Path must contain at least 2 points!");
  }

  float cte = 0;
  float min_dist = infinity();
  int closest_segment = last_index;
  Pose closest_point = path[closest_segment];


  for(int i = last_index; i<std::min(last_index+look_ahead, (int)path.size()-1); i++){
    Pose p1 = path[i];
    Pose p2 = path[i+1];

    float dx = p2.x-p1.x;
    float dy = p2.y-p1.y;
    float segment_length = hypot(dx,dy);

    if(segment_length < 0.0001) continue;

    float rx = pose.x - p1.x;
    float ry = pose.y - p1.y;

    float t = (rx*dx + ry*dy)/(powf(segment_length, 2));
    t = std::max(0.0f,std::min(1.0f,t));

    Pose proj = {p1.x + t * dx, p1.y + t * dy};

    float dist = hypot(pose.x-proj.x, pose.y-proj.y);
    if (dist < min_dist){
      min_dist = dist;
      closest_segment = i;
      double cross_product = (pose.x - p1.x) * dy - (pose.y - p1.y) * dx;
      cte = dist * (cross_product < 0 ? -1 : 1);
      closest_point = proj;
    }
  }
  return std::make_tuple(closest_segment, closest_point, cte);
}

float compute_stanley_heading(Pose pose, std::vector<Pose> path, int last_index, float velocity, bool forwards) {
  bool debug_mode = true; 
  auto [closest_segment, closest_point, cte] = crosstrack_error(pose, path, last_index, 3);

  // Check if we're near the end of the path
  bool approaching_end = ((path.size() - closest_segment) <= 3);

  Pose p1 = path[closest_segment];
  Pose p2 = path[closest_segment+1];
  
  // Look ahead to next segment to anticipate turns
  Pose p3 = (closest_segment + 2 < path.size()) ? path[closest_segment+2] : p2;
  
  // Calculate path angles in degrees
  float current_path_heading = to_deg(atan2(p2.y - p1.y, p2.x - p1.x));
  float next_path_heading = to_deg(atan2(p3.y - p2.y, p3.x - p2.x));

  // If approaching end, gradually transition to final heading
  if (approaching_end) {
    float end_heading = path.back().theta;  // Final desired heading
    float progress = (path.size() - closest_segment) / 4.0f;  // 1.0 at 4 segments from end, 0.0 at end
    current_path_heading = end_heading * (1.0f - progress) + current_path_heading * progress;
    next_path_heading = end_heading;
  }
  
  // Detect sharp turn (all angles in degrees)
  float turn_angle = fabs(reduce_negative_180_to_180(next_path_heading - current_path_heading));
  bool is_sharp_turn = turn_angle > 45.0;
  
  // Adaptive gains based on turn sharpness
  float k_base = 3.25;
  float k_adaptive;
  if (is_sharp_turn) {
    k_adaptive = k_base * (1.0 + turn_angle/45.0);
    velocity = std::min(velocity, 20.0f); // Limit speed in sharp turns
  } else {
    k_adaptive = k_base;
  }
  
  // Get current robot heading in degrees
  float robot_heading = reduce_negative_180_to_180(chassis.get_absolute_heading());
  
  // Calculate heading error in degrees
  float heading_error;
  if (is_sharp_turn && hypot(p2.x - pose.x, p2.y - pose.y) < 5.0) {
    heading_error = reduce_negative_180_to_180(next_path_heading - robot_heading);
  } else {
    heading_error = reduce_negative_180_to_180(current_path_heading - robot_heading);
  }
  
  // Calculate cross track error component
  // Convert CTE to heading correction in degrees
  float cte_heading = to_deg(atan2(k_adaptive * cte, std::max(velocity, 5.0f)));
  
  // Combine errors (all in degrees)
  float total_heading_error = reduce_negative_90_to_90(
     heading_error +  cte_heading
  );
  
  // Debug output
   if (debug_mode) {
    printf("Path heading: %.2f, Robot heading: %.2f\n", 
           current_path_heading, robot_heading);
    printf("CTE: %.2f in, CTE heading: %.2f\n", 
           cte, cte_heading);
    printf("Total heading error: %.2f\n", 
           total_heading_error);
  }
  
  return total_heading_error;  // Returns degrees
}

float trapezoidal_v(float c_velocity, float v_target, float a_max, float time){
  // If current velocity and target are very close, return target to avoid tiny oscillations
  if (fabs(c_velocity - v_target) < 0.1) {
    return v_target;
  }
  // Calculate maximum allowed velocity change based on acceleration and time
  float delta_v = a_max * time;
  
  // Use different smoothing factors for acceleration vs deceleration
  float accel_smoothing = 1.0;    // Normal acceleration smoothing
  float decel_smoothing = 1.5;    // More aggressive smoothing when slowing down
  
  // Apply acceleration limiting with direction-specific smoothing
  if (c_velocity < v_target) {
    // Accelerating
    delta_v *= accel_smoothing;
    float acceleration_factor = 1.0 - exp(-time / accel_smoothing);
    return c_velocity + std::min(delta_v, (v_target - c_velocity) * acceleration_factor);
  } else {
    // Decelerating - start slowing down earlier and more gradually
    delta_v *= decel_smoothing;
    float deceleration_factor = 1.0 - exp(-time / decel_smoothing);
    return c_velocity - std::min(delta_v, (c_velocity - v_target) * deceleration_factor);
  }
}

std::pair<float, float> calc_wheel_vel(float perc_vel, float steer_ang) {
    // Convert percentage velocity to a range of -1 to 1
    float lin_vel = perc_vel / 100.0f;

    // Compute angular velocity
    float ang_vel = lin_vel * tan(steer_ang) / chassis.wheel_base;

    // Compute unscaled wheel velocities
    float left_vel = lin_vel - (chassis.wheel_base / 2.0f) * ang_vel;
    float right_vel = lin_vel + (chassis.wheel_base / 2.0f) * ang_vel;

    // Determine max wheel velocity
    float max_wheel_vel = fmax(fabs(left_vel), fabs(right_vel));
    
    // Scale velocities if necessary
    if (max_wheel_vel > fabs(lin_vel)) {
        float scale = fabs(lin_vel) / fabs(max_wheel_vel);
        left_vel *= scale;
        right_vel *= scale;
    }
    // Return velocities as a pair
    return std::make_pair(left_vel * 100.0f, right_vel * 100.0f); // Convert back to percentage
}

void Drive::follow(std::vector<Pose> path, bool forwards) {
  int look_ahead = 3;
  int last_index = std::get<0>(crosstrack_error(pose,path,0,look_ahead));
  bool debug_mode = true;   
  float prev_velocity = 10;
  drive_settled = false;
  dist_travelled = 0;

  float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
  float average_position = start_average_position;
  
  // Initialize velocity and heading PIDs

  float dt = 0.01;
  float time_spent = 0;

  while(last_index < path.size() - 5) {

    average_position = (get_left_position_in()+get_right_position_in())/2.0;
    dist_travelled = -1*start_average_position + average_position;

    // Update position tracking
    last_index = std::get<0>(crosstrack_error(pose,path,last_index, look_ahead));

    // Get Target Velocity
    float target_velocity = path[last_index].theta;
    target_velocity = trapezoidal_v(prev_velocity, target_velocity, a_max, dt);
    prev_velocity = target_velocity;
    
    // Compute steering angle (radians)
    float steering_angle = to_rad(compute_stanley_heading(pose, path, last_index, target_velocity, forwards));
  
    auto [left_velocity, right_velocity] = calc_wheel_vel(target_velocity, steering_angle);
    if (forwards){
      drive_with_velocity(left_velocity, right_velocity);
    }


    if (time_spent > drive_timeout && !(drive_timeout == 0)){
      break;
    }
    task::sleep(10);
    time_spent += 10;
  }

  
  // Calculate final path heading between last two waypoints
  float final_path_heading = reduce_0_to_360(to_deg(atan2(
      path.back().x - path[path.size()-2].y,
      path.back().y - path[path.size()-2].x
    )));
  
  // Set up PIDs for final approach
  float dist_to_second_last = hypot(
    path[path.size()-2].x - pose.x,
    path[path.size()-2].y - pose.y
  );
  
  PID drivePID(dist_to_second_last, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID anglePID(reduce_negative_180_to_180(final_path_heading - get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  drivePID.time_spent_running = time_spent;

  // Line crossing detection setup
  bool line_settled = is_line_settled(path[path.size()-2].x, path[path.size()-2].y, final_path_heading, get_X_position(), get_Y_position());
  bool prev_line_settled = is_line_settled(path[path.size()-2].x, path[path.size()-2].y, final_path_heading, get_X_position(), get_Y_position());
  bool crossed_center_line = false;
  bool center_line_side = is_line_settled(path[path.size()-2].x, path[path.size()-2].y, final_path_heading+90, get_X_position(), get_Y_position());
  bool prev_center_line_side = center_line_side;
  
  while(!drivePID.is_settled()) {


    line_settled = is_line_settled(path[path.size()-2].x, path[path.size()-2].y, final_path_heading, get_X_position(), get_Y_position());
    if(line_settled && !prev_line_settled){ break; }
    prev_line_settled = line_settled;

    center_line_side = is_line_settled(path[path.size()-2].x, path[path.size()-2].y, final_path_heading+90, get_X_position(), get_Y_position());
    if(center_line_side != prev_center_line_side){
      crossed_center_line = true;
    }

    // Calculate distance and angle to target
    dist_to_second_last = dist_from_line(path[path.size()-2].x, path[path.size()-2].y, final_path_heading + 90, pose.x, pose.y);

    final_path_heading = reduce_0_to_360(to_deg(atan2(
      path.back().y - path[path.size()-2].y,
      path.back().x - path[path.size()-2].x
    )));

    float angle_error = reduce_negative_90_to_90(final_path_heading - get_absolute_heading());
    angle_error = forwards ? angle_error : angle_error+180;
    // Apply negative distance if crossed
    
    // Compute drive and angle outputs
    float drive_output = drivePID.compute(dist_to_second_last);
    drive_output = forwards ? drive_output : -drive_output;
    float angle_output = anglePID.compute(angle_error);
    
    // Apply voltage limits
    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    angle_output = clamp(angle_output, -turn_max_voltage, turn_max_voltage);
    
    // Scale drive output based on angle error
    float angle_from_dist = to_deg(atan2(path[path.size()-2].y - pose.y, path[path.size()-2].x - pose.x));
    float angle_scale = cos(to_rad(angle_from_dist - get_absolute_heading()));

    drive_output *= angle_scale;
    
    // Apply minimum voltage threshold
    if (fabs(dist_to_second_last) < drive_settle_error) {
      angle_output = 0;
    }
    
    // Drive outputs
    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    drive_output = clamp_min_voltage(drive_output, drive_min_voltage);

    drive_with_voltage(drive_output - angle_output, drive_output + angle_output);

    if (debug_mode) {
      printf("Distance: %f\n", dist_to_second_last);
      printf("Drive output: %f\n", drive_output);
      printf("X: %f, Y: %f\n", pose.x, pose.y);
    }

    if(drivePID.time_spent_running > drive_timeout && drive_timeout > 0){
      break;
    }
    task::sleep(10);
  }
  drive_settled = true;
  // Final stop
  if(drive_min_voltage == 0){
    drive_with_voltage(0, 0);
    DriveL.stop(brake);
    DriveR.stop(brake);
  }
  
  if (debug_mode) {
    printf("Final approach complete\n");
    printf("Final distance: %.2f\n", dist_to_second_last);
    printf("Final angle error: %.2f\n", reduce_negative_180_to_180(final_path_heading - get_absolute_heading()));
  }
}


void Drive::holonomic_drive_to_point(float X_position, float Y_position){
  holonomic_drive_to_point(X_position, Y_position, get_absolute_heading(), drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_point(float X_position, float Y_position, float angle){
  holonomic_drive_to_point(X_position, Y_position, angle, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_point(float X_position, float Y_position, float angle, float drive_max_voltage, float heading_max_voltage){
  holonomic_drive_to_point(X_position, Y_position, angle, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_point(float X_position, float Y_position, float angle, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  holonomic_drive_to_point(X_position, Y_position, angle, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_point(float X_position, float Y_position, float angle, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  desired_heading = angle;
  PID drivePID(hypot(X_position-get_X_position(),Y_position-get_Y_position()), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID turnPID(reduce_negative_180_to_180(to_deg(atan2(Y_position-get_Y_position(),X_position-get_X_position()))-get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  while(!(drivePID.is_settled() && turnPID.is_settled() ) ){
    float drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position());
    float turn_error = reduce_negative_180_to_180(to_deg(atan2(Y_position-get_Y_position(),X_position-get_X_position()))-get_absolute_heading());

    float drive_output = drivePID.compute(drive_error);
    float turn_output = turnPID.compute(turn_error);

    drive_output = clamp(drive_output, drive_max_voltage, drive_max_voltage);
    turn_output = clamp(turn_output, -heading_max_voltage, heading_max_voltage);

    float heading_error = atan2(Y_position-get_Y_position(),X_position-get_X_position());

    DriveLF.spin(fwd, drive_output*cos(to_rad(get_absolute_heading()) + heading_error - M_PI/4) + turn_output, volt);
    DriveLB.spin(fwd, drive_output*cos(-to_rad(get_absolute_heading()) - heading_error + 3*M_PI/4) + turn_output, volt);
    DriveRB.spin(fwd, drive_output*cos(to_rad(get_absolute_heading()) + heading_error - M_PI/4) - turn_output, volt);
    DriveRF.spin(fwd, drive_output*cos(-to_rad(get_absolute_heading()) - heading_error + 3*M_PI/4) - turn_output, volt);
    task::sleep(10);
  }
  DriveLF.stop(hold);
  DriveLB.stop(hold);
  DriveRB.stop(hold);
  DriveRF.stop(hold);
}

struct DrivePoint {
    double x;
    double y;
};

double a[] = {0,0,0,0};
double b[] = {0,0,0,0};
double m[] = {0,0,0,0};
double z[] = {0,0,0,0};

double C = 0;

DrivePoint P1 = {3,15};
DrivePoint P2 = {57.9,29.2};
DrivePoint P3 = {98.8,59.6};
DrivePoint P4 = {127,100};

double functionF(double input){
    double output;
    if((input >= P1.x) && (input < P2.x)){
        C = ((a[1]*powf(input - P2.x,2))*(input - P1.x)) + (b[1]*(input - P2.x)*powf(input - P1.x,2));
        output = (m[1]*(input - P1.x)) + P1.y + C;
    } else if ((input >= P2.x) && (input < P3.x)){
        C = ((a[2]*powf(input - P3.x,2))*(input - P2.x)) + (b[2]*(input - P3.x)*powf(input - P2.x,2));
        output = (m[2]*(input - P2.x)) + P2.y + C;
    } else if ((input >= P3.x) && (input <= P4.x)){
        C = ((a[3]*powf(input - P4.x,2))*(input - P3.x)) + (b[3]*(input - P4.x)*powf(input - P3.x,2));
        output = (m[3]*(input - P4.x)) + P4.y + C;
    } else {
        output = 0;
    }
    return output;
}

double driveCurveFunction(double input){
    // Derivatives
    m[1] = (P2.y - P1.y)/(P2.x - P1.x);
    m[2] = (P3.y - P2.y)/(P3.x - P2.x);
    m[3] = (P4.y - P3.y)/(P4.x - P3.x);
    //printf("\n m1: %f, m2: %f, m3: %f \n", m[1],m[2],m[3]);

    // Second Derivatives
    z[2] = 6*(((m[3]*P2.x) + (m[2]*P3.x) - (m[3]*P3.x) + 2*(m[2]*P4.x) + 2*(m[1]*P2.x) - 2*(m[1]*P4.x) - 3*(m[2]*P2.x))/(4*((P1.x*P2.x) + (P3.x*P4.x) - (P1.x*P4.x)) - powf(P2.x + P3.x, 2)));
    z[3] = 6*(((m[2]*P2.x) + (m[1]*P3.x) - (m[1]*P2.x) + 2*(m[2]*P1.x) + 2*(m[3]*P3.x) - 2*(m[3]*P1.x) - 3*(m[2]*P3.x))/(4*((P1.x*P2.x) + (P3.x*P4.x) - (P1.x*P4.x)) - powf(P2.x + P3.x, 2)));
    //printf("\n z2: %f, z3: %f \n", z[2], z[3]);

    // Constants
    a[1] = (z[2])/(6*(P1.x - P2.x));
    a[2] = ((2*z[2])+z[3])/(6*(P2.x-P3.x));
    a[3] = (z[3])/(3*(P3.x-P4.x));
    //printf("\n a1: %f, a2: %f, a3: %f \n", a[1],a[2],a[3]);
    
    b[1] = (2*z[2])/(6*(P2.x-P1.x));
    b[2] = ((2*z[3]) + z[2])/(6*(P3.x - P2.x));
    b[3] = (z[3])/(6*(P4.x-P3.x));
    //printf("\n b1: %f, b2: %f, b3: %f \n", b[1],b[2],b[3]);

    // Formula
    double output;
    if(input >= P1.x){
        output = functionF(input);
    } else if (input <= -P1.x){
        output = -functionF(-input);
    } else {
        output = 0;
    }
    //printf("\n Input: %f \n Output: %f \n -----", input, output);
    return output;
}

void Drive::control_arcade(){
  float throttle = deadband(controller(primary).Axis3.value(), 5);
  float turn = driveCurveFunction(controller(primary).Axis1.value());
  DriveL.spin(fwd, to_volt(throttle+turn), volt);
  DriveR.spin(fwd, to_volt(throttle-turn), volt);
}

void Drive::reverse_arcade(){
  float throttle = deadband(controller(primary).Axis2.value(), 5);
  float turn = driveCurveFunction(controller(primary).Axis4.value());
  DriveL.spin(fwd, to_volt(throttle+turn), volt);
  DriveR.spin(fwd, to_volt(throttle-turn), volt);
}

void Drive::single_arcade(){
  float throttle = deadband(controller(primary).Axis3.value(), 5);
  float turn = driveCurveFunction(controller(primary).Axis4.value());
  DriveL.spin(fwd, to_volt(throttle+turn), volt);
  DriveR.spin(fwd, to_volt(throttle-turn), volt);
}

void Drive::control_holonomic(){
  float throttle = deadband(controller(primary).Axis3.value(), 5);
  float turn = deadband(controller(primary).Axis1.value(), 5);
  float strafe = deadband(controller(primary).Axis4.value(), 5);
  DriveLF.spin(fwd, to_volt(throttle+turn+strafe), volt);
  DriveRF.spin(fwd, to_volt(throttle-turn-strafe), volt);
  DriveLB.spin(fwd, to_volt(throttle+turn-strafe), volt);
  DriveRB.spin(fwd, to_volt(throttle-turn+strafe), volt);
}

void Drive::control_tank(){
  float leftthrottle = deadband(controller(primary).Axis3.value(), 5);
  float rightthrottle = deadband(controller(primary).Axis2.value(), 5);
  DriveL.spin(fwd, to_volt(leftthrottle), volt);
  DriveR.spin(fwd, to_volt(rightthrottle), volt);
}

void Drive::capped_drive(float voltage){
  float throttle = to_volt(deadband(controller(primary).Axis3.value(), 5));
  float turn = to_volt(driveCurveFunction(controller(primary).Axis4.value()));
  throttle = clamp(throttle, -voltage, voltage);
  turn *= voltage / 12;
  DriveL.spin(fwd, throttle+turn, volt);
  DriveR.spin(fwd, throttle-turn, volt);
}

int Drive::position_track_task(){
  chassis.position_track();
  return(0);
}