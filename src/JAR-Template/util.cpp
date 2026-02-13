#include "vex.h"

float reduce_0_to_360(float angle) {
  while(!(angle >= 0 && angle < 360)) {
    if( angle < 0 ) { angle += 360; }
    if(angle >= 360) { angle -= 360; }
  }
  return(angle);
}

float reduce_negative_180_to_180(float angle) {
  while(!(angle >= -180 && angle < 180)) {
    if( angle < -180 ) { angle += 360; }
    if(angle >= 180) { angle -= 360; }
  }
  return(angle);
}

float reduce_negative_90_to_90(float angle) {
  while(!(angle >= -90 && angle < 90)) {
    if( angle < -90 ) { angle += 180; }
    if(angle >= 90) { angle -= 180; }
  }
  return(angle);
}

float to_rad(float angle_deg){
  return(angle_deg/(180.0/M_PI));
}

float to_deg(float angle_rad){
  return(angle_rad*(180.0/M_PI));
}

float clamp(float input, float min, float max){
  if( input > max ){ return(max); }
  if(input < min){ return(min); }
  return(input);
}

bool is_reversed(double input){
  if(input<0) return(true);
  return(false);
}

float to_volt(float percent){
  return(percent*12.0/100.0);
}

int to_port(int port){
  if(port>8){
    return(0);
  }
  return(port-1);
}

float deadband(float input, float width){
  if (fabs(input)<width){
    return(0);
  }
  return(input);
}

bool is_line_settled(float desired_X, float desired_Y, float desired_angle_deg, float current_X, float current_Y){
  return( (desired_X-current_X) * cos(to_rad(desired_angle_deg)) <= -(desired_Y-current_Y) * sin(to_rad(desired_angle_deg)) );
}

float left_voltage_scaling(float drive_output, float heading_output){
  float ratio = std::max(fabs(drive_output+heading_output), fabs(drive_output-heading_output))/12.0;
  if (ratio > 1) {
    return (drive_output+heading_output)/ratio;
  }
  return drive_output+heading_output;
}

float right_voltage_scaling(float drive_output, float heading_output){
  float ratio = std::max(fabs(drive_output+heading_output), fabs(drive_output-heading_output))/12.0;
  if (ratio > 1) {
    return (drive_output-heading_output)/ratio;
  }
  return drive_output-heading_output;
}

float clamp_min_voltage(float drive_output, float drive_min_voltage){
  if(drive_output < 0 && drive_output > -drive_min_voltage){
      return -drive_min_voltage;
  }
  if(drive_output > 0 && drive_output < drive_min_voltage){
    return drive_min_voltage;
  }
  return drive_output;
}

float dist_from_line(float X_position, float Y_position, float angle, float current_X, float current_Y) {
  // Convert angle to radians for trig functions
  float angle_rad = to_rad(angle);
  
  // Calculate line direction vector
  float dx = cos(angle_rad);
  float dy = sin(angle_rad);
  
  // Calculate vector from point on line to current position
  float px = current_X - X_position;
  float py = current_Y - Y_position;
  
  // Calculate perpendicular distance using cross product
  float distance = fabs(px * dy - py * dx);
  
  return distance;
}