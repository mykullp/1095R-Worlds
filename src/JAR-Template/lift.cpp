#include "vex.h"

Lift::Lift(enum::lift_setup lift_setup, motor_group LiftMotors, int tracker_port, float gear_ratio) :
  gear_ratio(gear_ratio),
  lift_setup(lift_setup),
  LiftMotors(LiftMotors),
  LiftTracker(rotation(tracker_port))  
{};
  
void Lift::spin_with_voltage(float voltage){
  LiftMotors.spin(fwd, voltage, volt);
}

double Lift::get_position_deg(){
  if (lift_setup == ONE_ROTATION){
    if(Rotation.position(degrees) < 0){
      return 0.01;
    } else {
      return(Rotation.position(degrees)/3);
    }
    //return(reduce_0_to_360(-Pot.angle(degrees)));
  } else {
    return(reduce_0_to_360(LiftMotors.position(degrees) * gear_ratio));
  }
}

void Lift::set_lift_constants(float max_voltage, float kp, float ki, float kd, float starti){
  this->max_voltage = max_voltage;
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
  this->starti = starti;
} 

void Lift::set_lift_exit_conditions(float settle_error, float settle_time, float timeout){
  this->settle_error = settle_error;
  this->settle_time = settle_time;
  this->timeout = timeout;
}

double settled_time, timedout;

void Lift::add_target(double angle){
  target.push_back(angle);
  //printf("PUSH BACK: %f\n",angle);
}

void Lift::replace_target(double angle){
  target[0] = angle;
  //printf("target: %f\n", target[0]);
  if(target.size() > 1){
    target.erase(target.begin()+1, target.end());
  }
}
void Lift::start_lift(float angle){
  add_target(angle);
  lift_task = task(pid_control);
}

bool isInDeadzone(float angle) {
  angle = fmod((angle + 360.0), 360.0);
  return angle >= state[3] && angle <= 360;
}

bool pathGoesThroughDeadzone(float current, float target) {
  float diff = reduce_negative_180_to_180(target - current);  // shortest path
  float step = 1.0;
  int steps = fabs(diff) / step;
  float direction = (diff > 0) ? step : -step;

  for (int i = 0; i <= steps; i++) {
    float intermediate = reduce_0_to_360(current+(i*direction));
    if (isInDeadzone(intermediate)) {
      return true;
    }
  }
  return false;
}

void Lift::lift_PID(){
  double target_angle = target[0];
  //printf("target: %f\n", target[0]);

  PID liftPID(reduce_negative_180_to_180((target_angle - get_position_deg())), kp, ki, kd, starti, settle_error, settle_time, timeout);
  float output;
  while(true){
    if(!(target.size() == 0)){
      if(liftPID.is_settled() == false && lift_pid){
        target_angle = target[0];
        float error = reduce_negative_180_to_180((target_angle - get_position_deg()));
        output = liftPID.compute(error);
        output = clamp(output, -max_voltage, max_voltage);

        if (fabs(error) < 10){
          output = clamp(output, -1.5, 1.5);
        }
        if (fabs(output) < 0.5){
          output = 0;
        }
        
        if(pathGoesThroughDeadzone(arm.get_position_deg(), target[0])){
          output *= -1;
        }

        spin_with_voltage(output);
      } else if (liftPID.is_settled() == true && lift_pid){
        output = 0;
        LiftMotors.stop(hold);

        if((target.size() > 1)){
          target_angle = target[0];
          target.erase(target.begin());
        }

        if (fabs(target[0] - get_position_deg()) > settle_error){
          liftPID.time_spent_settled = 0;
          liftPID.time_spent_running = 0;
        }
        liftPID.error = target_angle - get_position_deg();
      }
    }
    task::sleep(5);
  }
}

int Lift::pid_control(){
  arm.lift_PID();
  return(0);
}

