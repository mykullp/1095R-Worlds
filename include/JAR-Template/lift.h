#pragma once
#include "vex.h"

enum lift_setup {ONE_ROTATION, NO_ROTATION};

class Lift
{
private:
  float gear_ratio;

public: 
  lift_setup lift_setup;
  motor_group LiftMotors;
  rotation LiftTracker;
  
  float dist_travelled;

  float max_voltage;
  float kp;
  float ki;
  float kd;
  float starti;

  float settle_error;
  float settle_time;
  float timeout;

  bool lift_pid = true;

  std::vector<double> target;

  Lift(enum::lift_setup lift_setup, motor_group LiftMotors, int tracker_port, float gear_ratio);

  void spin_with_voltage(float voltage);

  double get_position_deg();

  void set_lift_constants(float max_voltage, float kp, float ki, float kd, float starti); 
  void set_lift_exit_conditions(float settle_error, float settle_time, float timeout);

  void add_target(double angle);

  void replace_target(double angle);

  void start_lift(float angle);

  void lift_PID();
  static int pid_control();
  vex::task lift_task;
};

