
#include "usercontrol.h"
#include "vex.h"

using namespace std; 
double loading = 72;
std::vector<double> state = {loading - 41,(loading),(loading + 125),(loading+ 198)};
double climb_pos = loading + 64;
double alliance_pos = loading + 177;

bool lift_move = false;
bool primed = false;
bool colorSort = true;
int count = 0;
bool intake_down = false;
int user_cont(){
    while(true){
        if (Controller1.ButtonR2.pressing() && !Controller1.ButtonR1.pressing()){
            if(arm.get_position_deg() <= state[1]+40 && tipper_out){
                tipper_out = false;
                arm.spin_with_voltage(-1);
                Tipper.set(false);
                task::sleep(200);
            }
            if ((arm.get_position_deg() <= state[1]+10)){ // SOFT STOP
                arm.max_voltage = 3;
                arm.lift_pid = true;
                arm.replace_target(state[0]);
            } else{ 
                arm.lift_pid = false;
                arm.spin_with_voltage(-12);
            }
        } else if(Controller1.ButtonR1.pressing() && (arm.get_position_deg() < state[3])){
            arm.lift_pid = false;
            arm.spin_with_voltage(12);
            if ((fabs(state[1] - arm.get_position_deg()) < 3)){ // REVERSE INTAKE FOR LOADING
                Intake.spinToPosition(Intake.position(degrees) - 100, degrees, true);
            }
        } else if (!arm.lift_pid && !lift_move){
            arm.lift_pid = false;
            if(arm.get_position_deg() > state[1] && arm.get_position_deg() < state[1] + 50){
                arm.LiftMotors.stop(hold);
            } else{ 
                arm.LiftMotors.stop(brake);
            }
        }
        // INTAKE CONTROL
        if(Controller1.ButtonL1.pressing()){ 
            Intake.spin(fwd, 12, volt);
            
            if (!(fabs(state[1] - arm.get_position_deg()) < 5)){ // No colorsort when loading
                color_sort();
            } else{
                if((Optical.color().hue() == 0 || Optical.color().hue() == 240) && Optical.isNearObject()){
                    if(Controller1.ButtonL1.pressing()){
                        task::sleep(400);
                        Intake.spin(reverse, 100, pct);
                        task::sleep(50);
                    }
                }
            }
            anti_jam();
        } else if (Controller1.ButtonL2.pressing()){ 
            Intake.spin(reverse,12,volt);
        } else if (intake_down){
            Intake.spin(reverse, 25, pct);
            task::sleep(300);
            //waitUntil(Optical2.color() == red && Optical2.isNearObject());
        Intake.stop(coast);
            intake_down = false;
        } else if (!lift_move){
            Intake.stop(coast);
        }
        task::sleep(5);
    }
}

bool clampDown = false;
bool armDown = false;
bool clawDown = false;
bool tipOut = false;
bool climb_toggle = false;
bool tipper_out = false;
int lift_macro(){
    while(true){
        if(Controller1.ButtonA.pressing()){ // MOVE LIFT TO LOADING
            arm.max_voltage = 12;
            arm.lift_pid = true;
            if(tipper_out){
                tipper_out = false;
                Tipper.set(false);
                task::sleep(200);
            }
            arm.replace_target(state[1]);
            waitUntil(!Controller1.ButtonA.pressing());
        }
        if(Controller1.ButtonRight.pressing()){ // MOVE LIFT TO DESCORE
            arm.max_voltage = 12;
            arm.lift_pid = true;
            arm.replace_target(state[2]);
            waitUntil(!Controller1.ButtonRight.pressing());
        }
        task::sleep(5);
    }
    return 0;
}

int pneumatics_toggle(){
    while(true){
        if(Controller1.ButtonX.pressing()){ // CLAMP 
            clampDown = !clampDown;
            Clamp.set(clampDown);
            waitUntil(!Controller1.ButtonX.pressing());
        }
        if(Controller1.ButtonY.pressing()){ // DOINKER
            armDown = !armDown;
            Arm.set(armDown);
            waitUntil(!Controller1.ButtonY.pressing());
        }
        if(Controller1.ButtonB.pressing()){ // CLAW
            clawDown = !clawDown;
            ArmClamp.set(clawDown);
            waitUntil(!Controller1.ButtonB.pressing());
        }
        if(Controller1.ButtonLeft.pressing()){ // COLORSORT TOGGLE 
            colorsort_on = !colorsort_on;
            Controller1.rumble("..");
            waitUntil(!Controller1.ButtonLeft.pressing());
        }
        if(Controller1.ButtonR1.pressing() && Controller1.ButtonR2.pressing()){ // TIPPER
            if(!tipper_out){
                tipper_out = true;
                if(tipper_out){
                    Controller1.rumble("..");
                }
                arm.spin_with_voltage(12);
                waitUntil(arm.get_position_deg() > state[1]+10);
                Tipper.set(tipper_out);
                waitUntil(!Controller1.ButtonR1.pressing() || !Controller1.ButtonR2.pressing());
            }
        }
        if(Controller1.ButtonUp.pressing()){ // CLIMB
            climb_toggle = !climb_toggle;
            arm.max_voltage = 12;
            arm.lift_pid = true;
            if(climb_toggle){
                Controller1.rumble("..");
                arm.replace_target(climb_pos);
            } if (!climb_toggle){
                arm.replace_target(state[0]);
            }
            waitUntil(!(Controller1.ButtonUp.pressing()));
        }
        task::sleep(5);
    }

}
