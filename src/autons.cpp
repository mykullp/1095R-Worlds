#include "vex.h"
double WALL = 71.125;

void timeDrive(double lspeed, double rspeed, double time)
{
    using namespace vex;
    chassis.drive_with_voltage(lspeed, rspeed);
    task::sleep(time);
    chassis.drive_with_voltage(0, 0);
}

float dist;
int run()
{
    chassis.drive_distance(dist);
    return 0;
}

bool runTimer = false;
double autonTime = 0;
int timers()
{
    while (true)
    {
        if (runTimer)
        {
            autonTime += 0.1;
            printf("Time: %f Angle:%f \n", autonTime, chassis.get_absolute_heading());
        }
        task::sleep(100);
    }
}

std::vector<Pose> path;
int follow_async()
{
    printf("running\n");
    chassis.follow(path, chassis.forwards);
    return 0;
}

int velocity_detector()
{
    double x_prev = chassis.get_X_position();
    double y_prev = chassis.get_Y_position();
    double x_vel, y_vel;
    while (true)
    {
        x_vel = fabs((fabs(chassis.get_X_position()) - x_prev) / 0.1);
        y_vel = fabs((fabs(chassis.get_Y_position()) - y_prev) / 0.1);
        if (!(x_vel == 0) && !(y_vel == 0) && ((y_vel > 5) || (x_vel > 5)))
        {
            if ((x_vel - y_vel) > 3)
            {
                if (TeamColor == 1)
                {
                    chassis.desired_heading = 330;
                }
                else
                {
                    chassis.desired_heading = 210;
                }
            }
            if ((y_vel - x_vel) > 3)
            {
                if (TeamColor == 1)
                {
                    chassis.desired_heading = 300;
                }
                else
                {
                    chassis.desired_heading = 240;
                }
            }
            else if (fabs(y_vel - x_vel) < 1)
            {
                if (TeamColor == 1)
                {
                    chassis.desired_heading = 315;
                }
                else
                {
                    chassis.desired_heading = 225;
                }
            }
        }

        x_prev = fabs(chassis.get_X_position());
        y_prev = fabs(chassis.get_Y_position());
        task::sleep(100);
    }
    return 0;
}

double intake_speed = 0;
bool intake_til_hold = false;
bool intake_til_down = false;
bool intake_til_scored = false;
bool lifting = false;
bool antijam_on = true;
bool colorsort_on = true;
double target;
double time_wait;
std::vector<double> color_detected;
double color_average;
void color_sort(){
    if (Optical.isNearObject() && colorsort_on){
        double intake_pos = Intake.position(deg);
        if (TeamColor == 1){
            if (Optical.color().hue() == 240){
                waitUntil(Intake.position(deg) > intake_pos + 290);
                Intake.spin(reverse, 100, pct);
                task::sleep(200);
            }
        }
        else if (TeamColor == -1){
            if (Optical.color().hue() == 0){
                waitUntil(Intake.position(deg) > intake_pos + 290);
                Intake.spin(reverse, 100, pct);
                task::sleep(200);
            }
        }
    }
}
void anti_jam(){
    if (fabs(Intake.velocity(rpm)) < 10 && Intake.torque(Nm) > 0.35 && fabs(arm.get_position_deg() - state[1]) > 5 && antijam_on){
        Intake.spin(reverse, 100, pct);
        task::sleep(90);
    }
}
int intake_auto(){
    auto_started = false;
    while (true){
        if (fabs(arm.LiftMotors.velocity(rpm)) > 10 && fabs(arm.get_position_deg() - state[1]) < 3 && arm.target[0] > state[1]){
            Intake.spinToPosition(Intake.position(degrees) - 100, degrees, true);
        }
        else if (intake_til_hold){
            Intake.spin(fwd, 60, pct);
            anti_jam();
            color_sort();
            if (Optical.isNearObject()){
                if (TeamColor == 1){
                    if ((Optical.color().hue() == 0) || !intake_til_hold){
                        intake_speed = 0;
                        Intake.stop(coast);
                        intake_til_hold = false;
                    }
                }
                else if (TeamColor == -1){
                    if ((Optical.color().hue() == 240) || !intake_til_hold){
                        intake_speed = 0;
                        Intake.stop(coast);
                        intake_til_hold = false;
                    }
                }
            }
        }
        else if (intake_til_down){
            Intake.spin(fwd, 25, pct);
            // waitUntil(Optical2.color() == red && Optical2.isNearObject());
            Intake.spinToPosition(Intake.position(degrees) - 35, degrees, true);
            Intake.stop(coast);
            intake_speed = 0;
            intake_til_down = false;
        }
        else if (intake_til_scored){
            Intake.spin(fwd, 12, volt);

            color_sort();
            anti_jam();
            if (Optical.isNearObject()){
                double intake_pos = Intake.position(deg);
                if (TeamColor == 1){
                    if (Optical.color().hue() == 0){
                        waitUntil(Intake.position(deg) > intake_pos + 510 || (fabs(Intake.velocity(rpm)) < 1 && Intake.torque() > 2.1));
                        Intake.stop(coast);
                        intake_speed = 0;
                        intake_til_scored = false;
                    }
                }
                else if (TeamColor == -1){
                    if (Optical.color() == blue){
                        waitUntil(Intake.position(deg) > intake_pos + 510 || (fabs(Intake.velocity(rpm)) < 1 && Intake.torque() > 2.1));
                        Intake.stop(coast);
                        intake_speed = 0;
                        intake_til_scored = false;
                    }
                }
            }
        }
        else if (!(intake_speed == 0) && !intake_til_hold){
            Intake.spin(fwd, (intake_speed / 100) * 12, volt);

            if (!(fabs(state[1] - arm.get_position_deg()) < 5)){ // No colorsort when loading
                color_sort();
            } else{
                if((Optical.color().hue() == 0 || Optical.color().hue() == 240) && Optical.isNearObject()){
                    if(!(intake_speed == 0)){
                        task::sleep(400);
                        Intake.spin(reverse, 50, pct);
                        task::sleep(50);
                    }
                }
            }
            anti_jam();
        }
        else{
            Intake.stop(coast);
        }
        if (auto_started){
            break;
        }
        task::sleep(5);
    }
    return 0;
}

bool intaking;
double intake_wait;
int lift_auto(){
    auto_started = false;
    while (true){
        if (lifting){
            task::sleep(time_wait);
            arm.add_target(target);
            lifting = false;
        }

        if (intaking){
            task::sleep(intake_wait);
            intake_speed = 100;
            intaking = false;
        }
        if (auto_started){
            break;
        }
        task::sleep(5);
    }
    return 0;
}

void move_intake(double speed, double wait){
    intake_wait = wait;
    intaking = true;
}

void prime_lift(){
    lifting = true;
    time_wait = 200;
    target = 90;
}

void move_lift(double Target, double Time_wait)
{
    target = Target;
    time_wait = Time_wait;
    lifting = true;
}

void default_constants(){
    chassis.set_drive_constants(10, 1, 0.05, 2, 2);
    chassis.set_heading_constants(12, 0.4, 0, 2, 0);
    chassis.set_turn_constants(12, 0.3, 0.01, 1.6, 5);
    chassis.set_swing_constants(12, 0.3, 0.03, 1.625, 5);

    chassis.set_drive_exit_conditions(0.5, 75, 2000);
    chassis.set_turn_exit_conditions(0.5, 75, 1500);
    chassis.set_swing_exit_conditions(2, 75, 1500);

    arm.set_lift_constants(12, 0.55, 0, 0.35, 10);
    arm.set_lift_exit_conditions(0.5, 300, 2000);
}

void autonSetup()
{
    using namespace vex;
    default_constants();
    vex::task run(timers);
    leftMotors.setStopping(hold);
    rightMotors.setStopping(hold);

    chassis.drive_min_voltage = 0;
    chassis.boomerang_lead = .45;
    chassis.boomerang_setback = 0;

    chassis.a_max = 20 * 1000;
    chassis.k_base = 3.5;
    chassis.max_velocity = 64.8;
    chassis.wheel_base = 12.5;

    colorsort_on = true;
}

void red_solo_six_wp(){
    TeamColor = 1;
    autonSetup();
    chassis.set_coordinates(-10.3, -50.8, 296.5);
    arm.start_lift(alliance_pos);
    task Timer(timers);
    task Intake_Auto(intake_auto);
    task Lift_Auto(lift_auto);
    runTimer = true;
    antijam_on = true;

    chassis.set_turn_exit_conditions(5, 50, 1500);
    chassis.set_drive_exit_conditions(1, 50, 2000);

    // Alliance Stake
    arm.replace_target(alliance_pos);
    chassis.drive_distance(6, chassis.get_absolute_heading(), 6, 6);

    // Goal 1
    chassis.drive_min_voltage = 3;
    chassis.straightline_to_point(-23, -24, false, 6, 6, 6, 5, 50, 2000);
    arm.replace_target(state[0]);
    Clamp.set(true);
    task::sleep(100);
    chassis.set_sideways(-1.55);

    // Ring 1-3
    /*
    chassis.drive_min_voltage = 4;
    chassis.turn_to_point(-40, -10, 0, 8, 5, 20, 1000);
    chassis.straightline_to_point(-40, -10, true);
    chassis.drive_timeout = 1000;
    chassis.drive_distance(20, 180, 10, 12);
    chassis.drive_timeout = 2000;
    chassis.drive_to_point(-24, -24, false);*/
    intake_speed = 100;
    chassis.turn_to_point(-46,-24);
    chassis.drive_to_point(-46, -24, true);
    chassis.drive_distance(-10);

    // Corner Rings
    antijam_on = false;
    chassis.drive_min_voltage = 5;
    chassis.drive_to_point(-50, -46, true, 7, 12, 12, 2, 25, 1000);
    chassis.drive_timeout = 1000;
    chassis.drive_distance(999, 180-315, 6, 8);
    antijam_on = false;
    chassis.drive_distance(-20, 180-315, 12, 12);
    antijam_on = true;
    chassis.drive_settle_error = 1;
    chassis.drive_distance(20, 180-315, 12, 12, 1, 10, 800);
    antijam_on = false;
    chassis.drive_min_voltage = 0;

    // Pickup Next Ring
    chassis.drive_to_point(-27, -48, false);
    chassis.turn_to_angle(0, 10);
    antijam_on = true;
    Clamp.set(false);
    chassis.set_sideways(0);
    chassis.drive_min_voltage = 0;
    intake_speed = 0;
    intake_til_scored = false;
    intake_til_hold = true;

    // Second Goal
    chassis.drive_timeout = 2000;
    chassis.drive_settle_error = 3;
    chassis.drive_to_point(33, -48, true, 0, 6.5, 6);
    chassis.drive_settle_error = 1;
    chassis.turn_settle_error = 5;
    chassis.turn_to_point(22, -24, 180);
    chassis.drive_min_voltage = 3;
    chassis.straightline_to_point(22, -24, false, 0, 6, 6);
    Clamp.set(true);
    clampDown = true;
    task::sleep(100);
    chassis.set_sideways(-1.55);

    // Last Ring + Touch
    chassis.turn_to_point(50, -30, 0, 12, 10, 0, 1000);
    intake_speed = 100;
    chassis.straightline_to_point(50, -30, true, 0, 10, 8);
    intake_til_hold = false;
    intake_til_scored = true;
    task::sleep(100);
    chassis.straightline_to_point(13, -13, false);
    Intake_Auto.stop();
    chassis.turn_to_angle(315);
    intake_speed = 30;
    runTimer = false;
}

void blue_solo_six_wp(){
    TeamColor = -1;
    autonSetup();
    chassis.set_coordinates(10.3, -50.8, 180-296.5);
    arm.start_lift(alliance_pos);
    task Timer(timers);
    task Intake_Auto(intake_auto);
    task Lift_Auto(lift_auto);
    runTimer = true;
    antijam_on = true;

    chassis.set_turn_exit_conditions(5, 50, 1500);
    chassis.set_drive_exit_conditions(1, 50, 2000);

    // Alliance Stake
    arm.replace_target(alliance_pos);
    chassis.drive_distance(6.5, chassis.get_absolute_heading(), 6, 6);

    // Goal 1
    chassis.drive_min_voltage = 3;
    chassis.straightline_to_point(23, -24, false, 6, 6, 6, 5, 50, 2000);
    arm.replace_target(state[0]);
    Clamp.set(true);
    task::sleep(100);
    chassis.set_sideways(-1.55);

    // Ring 1-3
    /*
    chassis.drive_min_voltage = 4;
    chassis.turn_to_point(-40, -10, 0, 8, 5, 20, 1000);
    chassis.straightline_to_point(-40, -10, true);
    chassis.drive_timeout = 1000;
    chassis.drive_distance(20, 180, 10, 12);
    chassis.drive_timeout = 2000;
    chassis.drive_to_point(-24, -24, false);*/
    intake_speed = 100;
    chassis.turn_to_point(46,-24);
    chassis.drive_to_point(46, -24, true);
    chassis.drive_distance(-10);

    // Corner Rings
    antijam_on = false;
    chassis.drive_min_voltage = 5;
    chassis.drive_to_point(50, -46, true, 7, 12, 12, 2, 25, 1000);
    chassis.drive_timeout = 1000;
    chassis.drive_distance(999, 315, 6, 8);
    antijam_on = false;
    chassis.drive_distance(-20, 315, 12, 12);
    antijam_on = true;
    chassis.drive_settle_error = 1;
    chassis.drive_distance(20, 315, 12, 12, 1, 10, 800);
    antijam_on = false;
    chassis.drive_min_voltage = 0;

    // Pickup Next Ring
    chassis.drive_to_point(27, -48, false);
    chassis.turn_to_angle(180-0, 10);
    antijam_on = true;
    Clamp.set(false);
    chassis.set_sideways(0);
    chassis.drive_min_voltage = 0;
    intake_speed = 0;
    intake_til_scored = false;
    intake_til_hold = true;

    // Second Goal
    chassis.drive_timeout = 2000;
    chassis.drive_settle_error = 3;
    chassis.drive_to_point(-33, -48, true, 0, 6.5, 6);
    chassis.drive_settle_error = 1;
    chassis.turn_settle_error = 5;
    chassis.turn_to_point(-23, -24, 180);
    chassis.drive_min_voltage = 3;
    chassis.straightline_to_point(-23, -24, false, 0, 6, 6);
    Clamp.set(true);
    clampDown = true;
    task::sleep(100);
    chassis.set_sideways(-1.55);

    // Last Ring + Touch
    chassis.turn_to_point(-50, -30, 0, 12, 10, 0, 1000);
    intake_speed = 100;
    chassis.straightline_to_point(-50, -30, true, 0, 10, 8);
    intake_til_hold = false;
    intake_til_scored = true;
    task::sleep(100);
    chassis.straightline_to_point(-13, -13, false);
    Intake_Auto.stop();
    chassis.turn_to_angle(180-315);
    intake_speed = 30;
    runTimer = false;
}

void red_pos_wp(){
    TeamColor = 1;
    autonSetup();
    chassis.set_coordinates(10.3, -50.8, 180-296.5);
    arm.start_lift(alliance_pos);
    task Timer(timers);
    task Intake_Auto(intake_auto);
    task Lift_Auto(lift_auto);
    runTimer = true;
    antijam_on = true;

    chassis.set_turn_exit_conditions(5, 50, 1500);
    chassis.set_drive_exit_conditions(1, 50, 2000);

    // Alliance Stake
    arm.replace_target(alliance_pos);
    chassis.drive_distance(6.5, chassis.get_absolute_heading(), 5, 6);

    // Pick Up Ring
    chassis.drive_distance(-15,225);
    chassis.turn_to_point(45.5,-24);
    intake_til_hold = true;
    arm.replace_target(state[0]);
    chassis.straightline_to_point(45.5,-24,true,0,8,8);

    // Score on Mogo
    chassis.turn_to_point(45.25,0,180);
    chassis.drive_distance(-16,chassis.get_absolute_heading(),4,6);
    Clamp.set(true);
    task::sleep(100);
    intake_til_hold = false;
    intake_speed = 100;
    chassis.drive_distance(12);
    Clamp.set(false);
    task::sleep(100);
    chassis.drive_distance(5);
    intake_speed = 0;

    // PickUp 2nd Goal
    chassis.turn_to_point(24,-24,180);
    chassis.straightline_to_point(24,-24,false,3,6,6);
    Clamp.set(true);
    chassis.set_sideways(-1.55);
    clampDown = true;
    task::sleep(100);

    // Corner
    chassis.turn_to_point(50,-50);
    intake_speed = 100;
    chassis.drive_min_voltage = 0;
    chassis.straightline_to_point(50,-50,true);
    task::sleep(500);
    chassis.drive_timeout = 1400;
    chassis.drive_distance(999, 315, 4, 8);
    antijam_on = false;
    chassis.drive_distance(-20, 315, 12, 12);
    antijam_on = true;
    chassis.drive_settle_error = 1;
    chassis.drive_distance(18, 315, 12, 12, 1, 10, 800);
    intake_til_scored = true;
    chassis.drive_timeout = 2000;
    chassis.drive_to_point(12.5,-12.5,false,0,7,8);
    runTimer = false;
}

void blue_pos_wp(){
    TeamColor = -1;
    autonSetup();
    chassis.set_coordinates(-10.3, -50.8, 296.5);
    arm.start_lift(alliance_pos);
    task Timer(timers);
    task Intake_Auto(intake_auto);
    task Lift_Auto(lift_auto);
    runTimer = true;
    antijam_on = true;

    chassis.set_turn_exit_conditions(5, 50, 1500);
    chassis.set_drive_exit_conditions(1, 50, 2000);

    // Alliance Stake
    arm.replace_target(alliance_pos);
    chassis.drive_distance(6, 296.5, 5, 6);

    // Pick Up Ring
    chassis.drive_distance(-15,315,10,12);
    arm.replace_target(state[0]);
    chassis.turn_to_point(-45.5,-24);
    intake_til_hold = true;
    chassis.straightline_to_point(-45.5,-24,true,0,8,8);

    // Score on Mogo
    chassis.turn_to_point(-45.5,0,180);
    chassis.drive_distance(-18.5,chassis.get_absolute_heading(),4,6);
    Clamp.set(true);
    task::sleep(100);
    intake_til_hold = false;
    intake_speed = 100;
    chassis.drive_distance(12);
    Clamp.set(false);
    task::sleep(100);
    chassis.drive_distance(5);
    intake_speed = 0;

    // PickUp 2nd Goal
    chassis.turn_to_point(-24,-24,180);
    chassis.straightline_to_point(-24,-24,false,3,6,6);
    Clamp.set(true);
    chassis.set_sideways(-1.55);
    clampDown = true;
    task::sleep(100);

    // Corner
    chassis.turn_to_point(-50,-50);
    intake_speed = 100;
    chassis.drive_min_voltage = 0;
    chassis.straightline_to_point(-50,-50,true);
    task::sleep(500);
    chassis.drive_timeout = 1400;
    chassis.drive_distance(999, 180-315, 4, 8);
    antijam_on = false;
    chassis.drive_distance(-20, 180-315, 12, 12);
    antijam_on = true;
    chassis.drive_settle_error = 1;
    chassis.drive_distance(18, 180-315, 12, 12, 1, 10, 800);
    antijam_on = false;
    intake_til_scored = true;
    chassis.drive_timeout = 2000;
    chassis.drive_to_point(-12.5,-12.5,false,0,7,8);
    runTimer = false;
}

void red_solo_wp_disrupt(){
    TeamColor = 1;
    autonSetup();
    chassis.set_coordinates(-10.3, -50.8, 296.5);
    arm.start_lift(alliance_pos);
    task Timer(timers);
    task Intake_Auto(intake_auto);
    task Lift_Auto(lift_auto);
    runTimer = true;
    antijam_on = true;

    chassis.set_turn_exit_conditions(5, 50, 1500);
    chassis.set_drive_exit_conditions(1, 50, 2000);

    // Alliance Stake
    arm.replace_target(alliance_pos);
    chassis.drive_distance(5.25, chassis.get_absolute_heading(), 6, 6);

    // Goal 1
    chassis.drive_min_voltage = 3;
    chassis.drive_max_voltage = 9;
    dist = -61;
    task drive1(run);
        waitUntil(chassis.dist_travelled < -28);
        Clamp.set(true);
        waitUntil(chassis.dist_travelled < -30);
        arm.replace_target(state[0]);
        chassis.desired_heading = 326;
        waitUntil(chassis.dist_travelled < -40);
        Clamp.set(false);
        waitUntil(chassis.dist_travelled < -59.5);
    drive1.stop();

    // Disrupt + Ring 1
    printf("Clamp Time: %f\n", autonTime);
    Clamp.set(true);
    chassis.set_sideways(-1.55);
    task::sleep(100);
    chassis.swing_settle_error = 10;
    chassis.left_swing_to_angle(220);
    intake_speed = 100;
    chassis.drive_to_point(-48, -24, true);
    chassis.drive_distance(-8,180);

    // Corner Rings
    antijam_on = false;
    chassis.drive_min_voltage = 5;
    chassis.turn_to_point(-50,-44,0,12,10,10,1000);
    chassis.drive_to_point(-50, -44, true, 7, 12, 12, 2, 25, 1000);
    chassis.turn_to_angle(180-315,12,20,10,1000);
    chassis.drive_timeout = 900;
    chassis.drive_distance(999, 180-315, 6, 8);
    antijam_on = false;
    chassis.drive_distance(-20, 180-315, 12, 12);
    antijam_on = true;
    chassis.drive_settle_error = 1;
    chassis.drive_distance(20, 180-315, 12, 12, 1, 10, 800);
    antijam_on = false;
    chassis.drive_min_voltage = 0;

    // Pickup Next Ring
    chassis.drive_to_point(-26, -48, false);
    chassis.turn_to_angle(0, 10);
    antijam_on = true;
    Clamp.set(false);
    chassis.set_sideways(0);
    chassis.drive_min_voltage = 0;
    intake_speed = 0;
    intake_til_scored = false;
    intake_til_hold = true;

    // Second Goal
    chassis.drive_timeout = 2000;
    chassis.drive_settle_error = 3;
    chassis.drive_to_point(33, -48, true, 0, 6.5, 6);
    chassis.drive_settle_error = 1;
    chassis.turn_settle_error = 5;
    chassis.turn_to_point(22, -24, 180);
    chassis.drive_min_voltage = 3;
    chassis.straightline_to_point(22, -24, false, 0, 6, 6);
    Clamp.set(true);
    clampDown = true;
    task::sleep(100);
    chassis.set_sideways(-1.55);

    // Last Ring + Touch
    chassis.turn_to_point(50, -30, 0, 12, 10, 0, 1000);
    intake_speed = 100;
    chassis.straightline_to_point(50, -30, true, 0, 10, 8);
    intake_til_hold = false;
    intake_til_scored = true;
    task::sleep(100);
    chassis.straightline_to_point(13.5, -13.5, false);
    Intake_Auto.stop();
    chassis.turn_to_angle(315);
    intake_speed = 30;
    runTimer = false;
}

void blue_solo_wp_disrupt(){
    TeamColor = -1;
    autonSetup();
    chassis.set_coordinates(10.3, -50.8, 180-296.5);
    arm.start_lift(alliance_pos);
    task Timer(timers);
    task Intake_Auto(intake_auto);
    task Lift_Auto(lift_auto);
    runTimer = true;
    antijam_on = true;

    chassis.set_turn_exit_conditions(5, 50, 1500);
    chassis.set_drive_exit_conditions(1, 50, 2000);

    // Alliance Stake
    arm.replace_target(alliance_pos);
    chassis.drive_distance(5.25, chassis.get_absolute_heading(), 6, 6);

    // Goal 1
    chassis.drive_min_voltage = 3;
    chassis.drive_max_voltage = 9;
    dist = -61;
    task drive1(run);
        chassis.desired_heading = 247;
        waitUntil(chassis.dist_travelled < -28);
        Clamp.set(true);
        waitUntil(chassis.dist_travelled < -30);
        arm.replace_target(state[0]);
        chassis.desired_heading = 180-326;
        waitUntil(chassis.dist_travelled < -40);
        Clamp.set(false);
        waitUntil(chassis.dist_travelled < -59.5);
    drive1.stop();

    // Disrupt + Ring 1
    printf("Clamp Time: %f\n", autonTime);
    Clamp.set(true);
    chassis.set_sideways(-1.55);
    task::sleep(100);
    chassis.swing_settle_error = 10;
    chassis.right_swing_to_angle(180-220);
    intake_speed = 100;
    chassis.drive_to_point(48, -24, true);
    chassis.drive_distance(-8,180-180);

    // Corner Rings
    antijam_on = false;
    chassis.drive_min_voltage = 5;
    chassis.turn_to_point(50,-44,0,12,10,10,1000);
    chassis.drive_to_point(50, -44, true, 7, 12, 12, 2, 25, 1000);
    chassis.turn_to_angle(315,12,20,10,1000);
    chassis.drive_timeout = 900;
    chassis.drive_distance(999, 315, 6, 8);
    antijam_on = false;
    chassis.drive_distance(-20, 315, 12, 12);
    antijam_on = true;
    chassis.drive_settle_error = 1;
    chassis.drive_distance(20, 315, 12, 12, 1, 10, 800);
    antijam_on = false;
    chassis.drive_min_voltage = 0;

    // Pickup Next Ring
    chassis.drive_to_point(26, -48, false);
    chassis.turn_to_angle(180-0, 10);
    antijam_on = true;
    Clamp.set(false);
    chassis.set_sideways(0);
    chassis.drive_min_voltage = 0;
    intake_speed = 0;
    intake_til_scored = false;
    intake_til_hold = true;

    // Second Goal
    chassis.drive_timeout = 2000;
    chassis.drive_settle_error = 3;
    chassis.drive_to_point(-33, -48, true, 0, 6.5, 6);
    chassis.drive_settle_error = 1;
    chassis.turn_settle_error = 5;
    chassis.turn_to_point(-22, -24, 180);
    chassis.drive_min_voltage = 3;
    chassis.straightline_to_point(-22, -24, false, 0, 6, 6);
    Clamp.set(true);
    clampDown = true;
    task::sleep(100);
    chassis.set_sideways(-1.55);

    // Last Ring + Touch
    chassis.turn_to_point(-50, -30, 0, 12, 10, 0, 1000);
    intake_speed = 100;
    chassis.straightline_to_point(-50, -30, true, 0, 10, 8);
    intake_til_hold = false;
    intake_til_scored = true;
    task::sleep(100);
    chassis.straightline_to_point(-13.5, -13.5, false);
    Intake_Auto.stop();
    chassis.turn_to_angle(180-315);
    intake_speed = 30;
    runTimer = false;
}

void red_solo_seven_wp(){
    TeamColor = 1;
    autonSetup();
    chassis.set_coordinates(-10.3, -50.8, 296.5);
    arm.start_lift(alliance_pos);
    task Timer(timers);
    task Intake_Auto(intake_auto);
    task Lift_Auto(lift_auto);
    runTimer = true;
    antijam_on = true;

    chassis.set_turn_exit_conditions(5, 50, 1500);
    chassis.set_drive_exit_conditions(1, 50, 2000);

    // Alliance Stake
    arm.replace_target(alliance_pos);
    chassis.drive_distance(6, chassis.get_absolute_heading(), 6, 6);

    // Goal 1
    chassis.straightline_to_point(-23, -24, false, 0, 7, 6, 5, 50, 2000);
    arm.replace_target(state[0]);
    Clamp.set(true);
    task::sleep(100);
    chassis.set_sideways(-1.55);

    // Ring 1-3
    intake_speed = 100;
    chassis.drive_min_voltage = 1;
    chassis.turn_to_point(-38, -10.5, 0, 12, 5, 10, 1000);
    chassis.straightline_to_point(-38, -10.5, true,3.5,12,12);
    chassis.drive_timeout = 700;
    //chassis.drive_distance(18, 180, 10, 12);
    chassis.drive_timeout = 2000;
    chassis.drive_to_point(-31, -24, false,0,10,12,2,10,1000);
    chassis.turn_to_point(-46,-24);
    chassis.straightline_to_point(-46, -24, true);
    chassis.drive_distance(-8,180,12,12,2,10,1000);

    // Corner Rings
    antijam_on = false;
    chassis.drive_min_voltage = 5;
    chassis.drive_to_point(-50, -45, true, 7, 12, 12, 2, 25, 1000);
    antijam_on = true;
    chassis.drive_timeout = 800;
    chassis.drive_distance(999, 180-315, 8, 8);
    antijam_on = false;
    chassis.drive_distance(-20, 180-315, 12, 12,3,10,1000);
    antijam_on = true;
    chassis.drive_settle_error = 1;
    chassis.drive_distance(18, 180-315, 12, 12, 3, 10, 800);
    antijam_on = false;
    chassis.drive_min_voltage = 0;

    // Pickup Next Ring
    chassis.drive_to_point(-26, -48, false);
    antijam_on = true;
    chassis.turn_to_angle(0, 10);
    Clamp.set(false);
    chassis.set_sideways(0);
    chassis.drive_min_voltage = 2;
    intake_speed = 0;
    intake_til_scored = false;
    intake_til_hold = true;

    // Second Goal
    chassis.drive_timeout = 2000;
    chassis.drive_settle_error = 3;
    chassis.drive_to_point(33, -48, true, 0, 6.75, 6);
    chassis.drive_settle_error = 1;
    chassis.turn_settle_error = 5;
    chassis.turn_to_point(22, -24, 180);
    chassis.drive_min_voltage = 3;
    chassis.straightline_to_point(22, -24, false, 0, 8, 6);
    Clamp.set(true);
    intake_til_hold = false;
    clampDown = true;
    task::sleep(100);
    chassis.set_sideways(-1.55);

    // Last Ring + Touch
    chassis.turn_to_point(50, -30, 0, 12, 10, 0, 1000);
    intake_speed = 100;
    chassis.straightline_to_point(50, -30, true, 0, 12, 10);
    intake_til_scored = true;
    task::sleep(100);
    chassis.straightline_to_point(13.5, -13.5, false,0,12,12);
    Intake_Auto.stop();
    chassis.turn_to_angle(315);
    intake_speed = 30;
    runTimer = false;
}

void blue_solo_seven_wp(){
    TeamColor = -1;
    autonSetup();
    chassis.set_coordinates(10.3, -50.8, -296.5);
    arm.start_lift(alliance_pos);
    task Timer(timers);
    task Intake_Auto(intake_auto);
    task Lift_Auto(lift_auto);
    runTimer = true;
    antijam_on = true;

    chassis.set_turn_exit_conditions(5, 50, 1500);
    chassis.set_drive_exit_conditions(1, 50, 2000);

    // Alliance Stake
    arm.replace_target(alliance_pos);
    chassis.drive_distance(6.5, chassis.get_absolute_heading(), 6, 6);

    // Goal 1
    chassis.straightline_to_point(23, -24, false, 0, 7, 6, 5, 50, 2000);
    arm.replace_target(state[0]);
    Clamp.set(true);
    task::sleep(100);
    chassis.set_sideways(-1.55);

    // Ring 1-3
    intake_speed = 100;
    chassis.drive_min_voltage = 1;
    chassis.turn_to_point(38, -10.5, 0, 12, 5, 10, 1000);
    chassis.straightline_to_point(38, -10.5, true,3.5,12,12);
    chassis.drive_timeout = 700;
    //chassis.drive_distance(18, 180, 10, 12);
    chassis.drive_timeout = 2000;
    chassis.drive_to_point(31, -24, false,0,10,12,2,10,1000);
    chassis.turn_to_point(46,-24);
    chassis.straightline_to_point(46, -24, true);
    chassis.drive_distance(-8,0,12,12,2,10,1000);

    // Corner Rings
    antijam_on = false;
    chassis.drive_min_voltage = 5;
    chassis.drive_to_point(50, -45, true, 7, 12, 12, 2, 25, 1000);
    antijam_on = true;
    chassis.drive_timeout = 800;
    chassis.drive_distance(999, 315, 8, 8);
    antijam_on = false;
    chassis.drive_distance(-20, 315, 12, 12,3,10,1000);
    antijam_on = true;
    chassis.drive_settle_error = 1;
    chassis.drive_distance(18, 315, 12, 12, 3, 10, 800);
    antijam_on = false;
    chassis.drive_min_voltage = 0;

    // Pickup Next Ring
    chassis.drive_to_point(26, -48, false);
    antijam_on = true;
    chassis.turn_to_angle(180-0, 10);
    Clamp.set(false);
    chassis.set_sideways(0);
    chassis.drive_min_voltage = 2;
    intake_speed = 0;
    intake_til_scored = false;
    intake_til_hold = true;

    // Second Goal
    chassis.drive_timeout = 2000;
    chassis.drive_settle_error = 3;
    chassis.drive_to_point(-33, -48, true, 0, 6.75, 6);
    chassis.drive_settle_error = 1;
    chassis.turn_settle_error = 5;
    chassis.turn_to_point(-22, -24, 180);
    chassis.drive_min_voltage = 3;
    chassis.straightline_to_point(-22, -24, false, 0, 8, 6);
    Clamp.set(true);
    intake_til_hold = false;
    clampDown = true;
    task::sleep(100);
    chassis.set_sideways(-1.55);

    // Last Ring + Touch
    chassis.turn_to_point(-50, -30, 0, 12, 10, 0, 1000);
    intake_speed = 100;
    chassis.straightline_to_point(-50, -30, true, 0, 12, 10);
    intake_til_scored = true;
    task::sleep(100);
    chassis.straightline_to_point(-13.5, -13.5, false,0,12,12);
    Intake_Auto.stop();
    chassis.turn_to_angle(180-315);
    intake_speed = 30;
    runTimer = false;
}

void red_goal_rush(){
    TeamColor = 1;
    autonSetup();
    chassis.set_coordinates(36.8, -47.3, 68.8);
    arm.start_lift(state[0]);
    task Timer(timers);
    task Intake_Auto(intake_auto);
    task Lift_Auto(lift_auto);
    runTimer = true;

    // Rush Goal
    intake_til_hold = true;
    chassis.drive_max_voltage = 12;
    Arm.set(true);
    dist = 999;
    task rush(run);
        waitUntil(chassis.dist_travelled > 10.5);
        ArmClamp.set(true);
        waitUntil(chassis.dist_travelled > 32.5 || chassis.drive_settled);
        ArmClamp.set(false);
        rush.stop();
    chassis.drive_timeout = 0;

    // Clamp Goal
    dist = -999;
    chassis.desired_heading = 77;
    chassis.drive_max_voltage = 12;
    task pullback(run);
        waitUntil(chassis.dist_travelled < -10);
        pullback.stop();
        chassis.drive_max_voltage = 8;
    task pullback2(run);
        waitUntil(chassis.get_Y_position() < -35); //-34
        pullback2.stop();
    chassis.drive_timeout = 1500; 
    chassis.drive_with_voltage(0, 0);

    ArmClamp.set(true);
    task::sleep(200);
    chassis.drive_distance(-8, chassis.get_absolute_heading());
    double goal_angle = chassis.get_absolute_heading() + 190;
    chassis.turn_to_angle(goal_angle, 10, 5, 25, 1000);
    Arm.set(false);
    chassis.drive_distance(-27.5, goal_angle, 6, 6);
    Clamp.set(true);
    chassis.set_sideways(-1.55);
    task::sleep(200);

    // Intake Ring onto Goal
    /*if(Optical3.color().hue() > 80 && Optical3.color().hue() < 97){
        intake_speed = 100;
    }*/
    intake_til_hold = false;
    intake_speed = 100;
    chassis.drive_min_voltage = 3;
    chassis.drive_distance(11);
    intake_til_hold = true;
    chassis.drive_min_voltage = 0;
    chassis.drive_to_point(23, -47, true);
    chassis.turn_to_angle(60,12,10,10,1000);

    // Clamp 2nd Goal
    Clamp.set(false);
    chassis.set_sideways(0);
    task::sleep(100);
    chassis.drive_distance(5);
    chassis.turn_to_point(24, -19, 180, 10, 5, 25, 1000);
    chassis.straightline_to_point(24, -19, false, 0, 6, 6);
    clampDown = true;
    Clamp.set(true);
    ArmClamp.set(false);
    chassis.set_sideways(-1.55);
    task::sleep(100);

    // Intake Preload
    chassis.drive_min_voltage = 4;
    chassis.turn_settle_error = 15;
    chassis.drive_max_voltage = 12;
    chassis.drive_max_voltage = 8;
    chassis.turn_to_point(48, -46, 4, 12, 20, 0, 1000);
    intake_til_hold = false;
    intake_speed = 100;
    chassis.drive_to_point(48, -46, true);
    chassis.turn_to_angle(315);
    chassis.drive_min_voltage = 0;

    // First Jam
    chassis.drive_timeout = 1000;
    antijam_on = false;
    dist = 999;
    chassis.drive_max_voltage = 5.75;
    chassis.heading_max_voltage = 12;
    task headingdetector(velocity_detector);
    task corner(run);
    waitUntil(chassis.drive_settled);
    corner.stop();
    headingdetector.stop();
    chassis.drive_max_voltage = 10;
    Arm.set(true);
    chassis.drive_distance(-23, 315, 12, 12);
    antijam_on = true;
    chassis.drive_settle_error = 1;
    chassis.drive_distance(18);
    chassis.turn_to_angle(240, 12, 10, 0, 1000);
    Arm.set(false);
    chassis.drive_to_pose(48, -22, 270, false);

    runTimer = false;
}

void blue_goal_rush(){
    TeamColor = -1;
    autonSetup();
    chassis.set_coordinates(-60, -49, 66);
    arm.start_lift(state[0]);
    task Timer(timers);
    task Intake_Auto(intake_auto);
    task Lift_Auto(lift_auto);
    runTimer = true;

    // Rush Goal
    intake_til_hold = true;
    chassis.drive_max_voltage = 12;
    Arm.set(true);
    dist = 999; // 40
    task rush(run);
    waitUntil(chassis.dist_travelled > 10.5);
    ArmClamp.set(true);
    waitUntil(chassis.dist_travelled > 31.5 || chassis.drive_settled);
    ArmClamp.set(false);
    rush.stop();
    chassis.drive_timeout = 0;

    // Clamp Goal
    dist = -999;
    chassis.desired_heading = 76;
    chassis.drive_max_voltage = 12;
    task pullback(run);
    waitUntil(chassis.dist_travelled < -10);
    pullback.stop();
    chassis.drive_max_voltage = 8;
    task pullback2(run);
    waitUntil(chassis.get_Y_position() < -33); //-30
    pullback2.stop();
    chassis.drive_timeout = 1500;
    chassis.DriveL.stop(brake);
    chassis.DriveR.stop(brake);

    ArmClamp.set(true);
    task::sleep(200);
    chassis.drive_distance(-6, chassis.get_absolute_heading());
    double goal_angle = chassis.get_absolute_heading() + 190;
    Arm.set(false);
    task::sleep(200);
    chassis.turn_to_angle(180, 12, 40, 15, 500); // turn to bump the red ring if necessary
    chassis.turn_to_angle(goal_angle, 12, 5, 25, 1000);
    chassis.drive_distance(-25, goal_angle, 6, 6);
    Clamp.set(true);
    chassis.set_sideways(-1.55);
    task::sleep(200);

    // Intake Ring onto Goal
    /*if(Optical3.color().hue() > 85 && Optical3.color().hue() < 95){
        intake_speed = 100;
    }*/
    chassis.turn_to_point(-34, -49, 0, 12, 10, 0, 1000);
    intake_speed = 100;
    chassis.drive_to_point(-34, -49, true, 0, 12, 12, 2, 25, 2000);
    intake_speed = 0;
    chassis.turn_to_angle(120, 12, 10, 10, 800);

    // Clamp 2nd Goal
    intake_til_hold = false;
    Clamp.set(false);
    ArmClamp.set(false);
    chassis.set_sideways(0);
    task::sleep(100);
    chassis.drive_distance(5);
    chassis.turn_to_point(-23.5, -25, 180, 12, 5, 25, 1000);
    chassis.straightline_to_point(-23.5, -25, false, 0, 6, 6);
    clampDown = true;
    Clamp.set(true);
    chassis.set_sideways(-1.55);
    task::sleep(100);

    // intake preload
    intake_speed = 100;
    chassis.turn_settle_error = 15;
    chassis.boomerang_lead = 0.45;
    chassis.drive_min_voltage = 3;
    chassis.drive_to_pose(-55,-46,250,true);
    chassis.drive_min_voltage = 0;
    chassis.drive_distance(10);
    chassis.drive_min_voltage = 3;
    task::sleep(300);
    chassis.drive_distance(-7,225,12,12,1,10,1000);

    // First Jam
    chassis.drive_timeout = 1000;
    dist = 999; 
    chassis.drive_max_voltage = 5;
    chassis.heading_max_voltage = 12;
    antijam_on = false;
    task corner(run);
    waitUntil(chassis.drive_settled);
    corner.stop();
    chassis.drive_max_voltage = 10;
    // Arm.set(true);
    chassis.drive_distance(-20, 180 - 315, 4, 12);
    antijam_on = true;
    chassis.drive_settle_error = 1;
    chassis.drive_distance(18, 225, 12, 12, 1, 10, 800);
    // chassis.turn_to_angle(170, 12, 10, 0, 800);
    // Arm.set(false);
    chassis.drive_min_voltage = 0;
    chassis.drive_to_pose(-48, -22, 270, false);

    runTimer = false;
}

void red_goal_rush_wall_disrupt(){
    TeamColor = 1;
    autonSetup();
    chassis.set_coordinates(36.8, -47.3, 68.8);
    arm.start_lift(state[0]);
    task Timer(timers);
    task Intake_Auto(intake_auto);
    task Lift_Auto(lift_auto);
    runTimer = true;

    // Rush Goal
    intake_til_hold = true;
    chassis.drive_max_voltage = 12;
    Arm.set(true);
    dist = 999;
    task rush(run);
    waitUntil(chassis.dist_travelled > 10.5);
    ArmClamp.set(true);
    waitUntil(chassis.dist_travelled > 32.5 || chassis.drive_settled);
    ArmClamp.set(false);
    rush.stop();
    chassis.drive_timeout = 0;

    // Clamp Goal
    dist = -999;
    chassis.desired_heading = 77;
    chassis.drive_max_voltage = 12;
    task pullback(run);
    waitUntil(chassis.dist_travelled < -10);
    pullback.stop();
    chassis.drive_max_voltage = 8;
    task pullback2(run);
    waitUntil(chassis.get_Y_position() < -35); //-34
    pullback2.stop();
    chassis.drive_timeout = 1500;
    chassis.drive_with_voltage(0, 0);

    ArmClamp.set(true);
    task::sleep(200);
    chassis.drive_distance(-8, chassis.get_absolute_heading());
    double goal_angle = chassis.get_absolute_heading() + 190;
    chassis.turn_to_angle(goal_angle, 10, 5, 25, 1000);
    Arm.set(false);
    chassis.drive_distance(-27.5, goal_angle, 6, 6);
    Clamp.set(true);
    chassis.set_sideways(-1.55);
    task::sleep(200);

    // Intake Ring onto Goal
    /*if(Optical3.color().hue() > 80 && Optical3.color().hue() < 97){
        intake_speed = 100;
    }*/
    intake_til_hold = false;
    intake_speed = 100;
    chassis.drive_min_voltage = 3;
    chassis.drive_distance(11);
    intake_til_hold = true;
    chassis.drive_min_voltage = 0;
    chassis.drive_to_point(23, -47, true);
    chassis.turn_to_angle(60,12,10,10,1000);

    // Clamp 2nd Goal
    Clamp.set(false);
    chassis.set_sideways(0);
    task::sleep(100);
    chassis.drive_distance(5);
    chassis.turn_to_point(24, -19, 180, 10, 5, 25, 1000);
    chassis.straightline_to_point(24, -19, false, 0, 6, 6);
    clampDown = true;
    Clamp.set(true);
    ArmClamp.set(false);
    chassis.set_sideways(-1.55);
    task::sleep(100);

    // Intake Preload
    chassis.drive_min_voltage = 4;
    chassis.turn_settle_error = 15;
    chassis.drive_max_voltage = 12;
    chassis.drive_max_voltage = 8;
    chassis.turn_to_point(48, -46, 4, 12, 20, 0, 1000);
    intake_til_hold = false;
    intake_speed = 100;
    chassis.drive_to_point(48, -46, true);
    chassis.turn_to_angle(315);
    chassis.drive_min_voltage = 0;

    // First Jam
    chassis.drive_timeout = 1000;
    antijam_on = false;
    dist = 999;
    chassis.drive_max_voltage = 5.75;
    chassis.heading_max_voltage = 12;
    task headingdetector(velocity_detector);
    task corner(run);
    waitUntil(chassis.drive_settled);
    corner.stop();
    headingdetector.stop();
    chassis.drive_max_voltage = 10;
    
    // Disrupt Wall
    chassis.drive_min_voltage = 0;
    chassis.drive_to_point(62,-38,false,0,8,8);
    chassis.turn_to_angle(315,12,30,0,1000);
    chassis.turn_to_angle(45);
    arm.replace_target(state[2]-15);
    dist = 999;
    chassis.drive_max_voltage = 4;
    task drive1(run);
        waitUntil(chassis.dist_travelled > 10);
        //Tipper.set(true);
        waitUntil(Right_Line.reflectivity(percent) > 20 || Left_Line.reflectivity(percent) > 20);
        chassis.DriveL.stop(hold);
        chassis.DriveR.stop(hold);
        drive1.stop();
    runTimer = false;
}

void blue_goal_rush_wall_disrupt(){
    TeamColor = -1;
    autonSetup();
    chassis.set_coordinates(-60, -49, 66);
    arm.start_lift(state[0]);
    task Timer(timers);
    task Intake_Auto(intake_auto);
    task Lift_Auto(lift_auto);
    runTimer = true;

    // Rush Goal
    intake_til_hold = true;
    chassis.drive_max_voltage = 12;
    Arm.set(true);
    dist = 999; // 40
    task rush(run);
    waitUntil(chassis.dist_travelled > 10.5);
    ArmClamp.set(true);
    waitUntil(chassis.dist_travelled > 31.5 || chassis.drive_settled);
    ArmClamp.set(false);
    rush.stop();
    chassis.drive_timeout = 0;

    // Clamp Goal
    dist = -999;
    chassis.desired_heading = 76;
    chassis.drive_max_voltage = 12;
    task pullback(run);
    waitUntil(chassis.dist_travelled < -10);
    pullback.stop();
    chassis.drive_max_voltage = 8;
    task pullback2(run);
    waitUntil(chassis.get_Y_position() < -33); //-30
    pullback2.stop();
    chassis.drive_timeout = 1500;
    chassis.DriveL.stop(brake);
    chassis.DriveR.stop(brake);

    ArmClamp.set(true);
    task::sleep(200);
    chassis.drive_distance(-6, chassis.get_absolute_heading());
    double goal_angle = chassis.get_absolute_heading() + 190;
    Arm.set(false);
    task::sleep(200);
    chassis.turn_to_angle(180, 12, 40, 15, 500); // turn to bump the red ring if necessary
    chassis.turn_to_angle(goal_angle, 12, 5, 25, 1000);
    chassis.drive_distance(-25, goal_angle, 6, 6);
    Clamp.set(true);
    chassis.set_sideways(-1.55);
    task::sleep(200);

    // Intake Ring onto Goal
    /*if(Optical3.color().hue() > 85 && Optical3.color().hue() < 95){
        intake_speed = 100;
    }*/
    chassis.turn_to_point(-34, -49, 0, 12, 10, 0, 1000);
    intake_speed = 100;
    chassis.drive_to_point(-34, -49, true, 0, 12, 12, 2, 25, 2000);
    intake_speed = 0;
    chassis.turn_to_angle(120, 12, 10, 0, 800);

    // Clamp 2nd Goal
    intake_til_hold = false;
    Clamp.set(false);
    ArmClamp.set(false);
    chassis.set_sideways(0);
    task::sleep(100);
    chassis.drive_distance(5);
    chassis.turn_to_point(-23.5, -25, 180, 12, 5, 25, 1000);
    chassis.straightline_to_point(-23.5, -25, false, 0, 6, 6);
    clampDown = true;
    Clamp.set(true);
    chassis.set_sideways(-1.55);
    task::sleep(100);

    // intake preload
    intake_speed = 100;
    chassis.turn_settle_error = 15;
    chassis.boomerang_lead = 0.45;
    chassis.drive_min_voltage = 3;
    chassis.drive_to_pose(-55,-46,250,true);
    chassis.drive_min_voltage = 0;
    chassis.drive_distance(10);
    chassis.drive_min_voltage = 3;
    task::sleep(300);
    //chassis.drive_distance(-7,225,12,12,1,10,1000);

    // First Jam
    chassis.drive_timeout = 1000;
    dist = 999; 
    chassis.drive_max_voltage = 9;
    chassis.heading_max_voltage = 12;
    antijam_on = false;
    task corner(run);
    waitUntil(chassis.drive_settled);
    corner.stop();
    chassis.drive_max_voltage = 10;
    
    // Disrupt Wall
    chassis.drive_min_voltage = 0;
    chassis.drive_to_point(-62,-33,false,0,8,8);
    chassis.turn_to_angle(180-315,12,30,0,1000);
    chassis.turn_to_angle(125);
    arm.replace_target(state[2]-15);
    dist = 999;
    chassis.drive_max_voltage = 4;
    task drive1(run);
        waitUntil(chassis.dist_travelled > 10);
        //Tipper.set(true);
        waitUntil(Right_Line.reflectivity(percent) > 20 || Left_Line.reflectivity(percent) > 20);
        chassis.DriveL.stop(hold);
        chassis.DriveR.stop(hold);
        drive1.stop();

    runTimer = false;
}

void skills(){
    TeamColor = 1;
    autonSetup();
    chassis.set_coordinates(0,-61,90);
    arm.start_lift(state[0]);
    task Timer(timers);
    task Intake_Auto(intake_auto);
    task Lift_Auto(lift_auto);
    runTimer = true;

    chassis.set_turn_exit_conditions(3, 50, 1500);
    chassis.set_drive_constants(10, 1.15, 0.05, 5, 2);
    chassis.set_turn_constants(12, 0.22, 0.03, 1,5); //0.0275

    //Score Alliance Stake
    intake_speed = 100;
    task::sleep(600);
    intake_speed = 0;

    //Grab Goal #1
    chassis.straightline_to_point(0,-47,true);
    chassis.turn_to_angle(180);
    chassis.drive_distance(-21,180,6,6);
    Clamp.set(true);
        chassis.set_sideways(-1.55);
    task::sleep(100);

    //Intake Ring #1
    intake_speed = 100;
    chassis.turn_to_point(24,-25,0,12,5,30,1000);
        chassis.drive_min_voltage = 6;
    chassis.straightline_to_point(24,-25,true);

    //Intake Ring to Lift
        chassis.heading_max_voltage = 6;
        chassis.drive_max_voltage = 8;
    move_lift(state[1],900);
    chassis.drive_to_pose(45,22,90,true);
    chassis.drive_distance(5);
        chassis.drive_min_voltage = 0;
        chassis.boomerang_lead = 0;
        chassis.drive_timeout = 1000;
    chassis.turn_to_point(42,1,180);
    chassis.straightline_to_point(42,1,false);
        chassis.drive_timeout = 2000;
        chassis.heading_max_voltage = 2;

    //Wall Stake and Intake Ring #2
        chassis.drive_max_voltage = 10;
    arm.replace_target(state[1]+40);
    chassis.turn_to_point(999,0);

    intake_til_scored = true;
        chassis.heading_max_voltage = 10;
        chassis.drive_timeout = 600;
    chassis.drive_distance(999,chassis.get_absolute_heading(),6,0);
    intake_speed = 100;
    chassis.drive_with_voltage(3,3);
    arm.replace_target(state[2]);
    task::sleep(550);
    chassis.drive_with_voltage(0,0);
        chassis.drive_timeout = 3000;

    chassis.straightline_to_point(46,1,false);
    move_lift(state[0],300);

    //Intake Ring #4-6
    chassis.turn_to_angle(270);
    reset_coordinates(false,true);
    task::sleep(25);
    intake_til_scored = false;
    intake_speed = 100;
        chassis.drive_max_voltage = 6.5;
        chassis.drive_min_voltage = 0;
    chassis.drive_to_point(46,-57,true);
    chassis.turn_to_angle(270,12,1,25,1000);
    reset_coordinates(false,true);
        chassis.turn_max_voltage = 12;
    chassis.turn_to_point(59,-44,-15,5,10,20,800);
        chassis.boomerang_lead = 0.55;
        chassis.drive_timeout = 800;
    chassis.drive_to_pose(59,-30,90,true);
        chassis.drive_timeout = 2500;
    task::sleep(300);
        chassis.drive_timeout = 650;
        chassis.desired_heading = 107;

    //Drop Mogo
    dist = -999;
    task drop1(run);
        waitUntil(chassis.dist_travelled < -9 || chassis.drive_settled);
        intake_speed = -50;
        Clamp.set(false);
        task::sleep(50);
        chassis.set_sideways(0);
        waitUntil(chassis.drive_settled);
        drop1.stop();

    intake_speed = 0;
        chassis.drive_timeout = 2500;

    //Intake Ring from Under Blue
    intake_speed = 100;
        chassis.drive_max_voltage = 10;
        chassis.boomerang_lead = 0.3;
        chassis.heading_max_voltage = 8.5;

        chassis.drive_settle_error = 31;
    arm.replace_target(state[1]);
    intake_speed = 100;
    chassis.drive_to_point(48,42.25,true);
        chassis.drive_settle_error = 0.5;
    chassis.drive_to_point(48,42.25,true,0,4,4);
    chassis.heading_max_voltage = 12;
    Arm.set(true);

    //Get Mogo #2 in the Corner
        chassis.drive_max_voltage = 10;
        chassis.heading_max_voltage = 3;
    chassis.drive_distance(-2.5);
        chassis.heading_max_voltage = 12;
    chassis.turn_to_point(22.5,61,180);
    Arm.set(false);
        chassis.drive_max_voltage = 6;
        chassis.heading_max_voltage = 3;
        chassis.drive_settle_error = 2;
    chassis.straightline_to_point(22.5,61,false,3,6,4,1.5,75,1000);
        chassis.drive_settle_error = 1;
        chassis.drive_max_voltage = 12;

    Clamp.set(true);
        chassis.set_sideways(-1.55);
    task::sleep(100);

    chassis.turn_to_angle(270,10,1,25,1000);
    reset_coordinates(true,true);
    task::sleep(50);
    chassis.turn_to_angle(228);
    intake_speed = 0;
    Clamp.set(false);
    task::sleep(100);
    chassis.set_sideways(0);
        chassis.drive_timeout = 400;
    chassis.drive_distance(-999,228,8,8);
        chassis.drive_timeout = 1000;
    chassis.drive_distance(-999,200,6,6);
        chassis.drive_timeout = 2000;

    chassis.drive_settle_error = 0.5;
    chassis.drive_to_point(26,48,true,0,8,8);

    //Put Ring on Alliance
    chassis.turn_to_angle(0);
        chassis.drive_max_voltage = 9;
    chassis.drive_distance(-33,0,6,6);
    intake_speed = 0;
    reset_coordinates(true,true);
    Clamp.set(true);
    chassis.set_sideways(-1.55);
    task::sleep(100);
        chassis.drive_timeout = 2000;
    chassis.straightline_to_point(0,48,true,0,5,8);
    chassis.turn_to_point(0,70);
    chassis.drive_timeout = 800;
    chassis.drive_distance(999,chassis.get_absolute_heading(),5,0);
    chassis.drive_timeout = 2000;
    arm.replace_target(alliance_pos);
    chassis.drive_distance(-7.5,chassis.get_absolute_heading(),5,5);
    task::sleep(100);

    chassis.drive_min_voltage = 4;
    chassis.drive_distance(-15,90,12,12);
    //Intake Ring #1 - 3
    chassis.turn_to_point(23,24);
    arm.replace_target(state[0]);
    intake_speed = 100;
    chassis.drive_min_voltage = 0;
    chassis.straightline_to_point(23,24,true,0,8,8);
        chassis.drive_max_voltage = 8;
        chassis.heading_max_voltage = 12;
    intake_til_scored = true;
        double current_time = autonTime;
    chassis.turn_to_point(0,0,0,6,3,25,1000);
    waitUntil(!intake_til_scored || autonTime > current_time + 0.8);
    intake_til_scored = false;
    task::sleep(200);
        chassis.drive_min_voltage = 4;
        chassis.boomerang_lead = 0;

    dist = 999;
    task drive1(run);
        waitUntil(chassis.dist_travelled > 13);
        Intake_Auto.stop();
        Intake.spinToPosition(Intake.position(degrees)+180*6,degrees, false);
        waitUntil(chassis.dist_travelled > 15);
        intake_speed = 0;
    drive1.stop();

    //Intake Ring #4-6
        chassis.drive_settle_error = 1;
        chassis.drive_settle_time = 25;
        chassis.turn_settle_error = 3;
    chassis.drive_to_point(-16,-16,true);
    task Intake_auto(intake_auto);
    intake_speed = 100;

        chassis.drive_max_voltage = 7;
        chassis.drive_timeout = 3000;
        chassis.drive_min_voltage = 0;
    chassis.drive_to_point(-47.5,-47.5,true);
    chassis.turn_to_point(-48,-58);
    chassis.drive_to_point(-48,-58,true);

    chassis.turn_to_point(-61,-50,15,7,10,20,800);
        chassis.boomerang_lead = 0.55;
        chassis.drive_timeout = 800;
        chassis.drive_timeout = 1200;
        chassis.drive_settle_error = 2;
    chassis.straightline_to_point(-61,-50,true);
        chassis.drive_timeout = 2500;
    chassis.turn_to_angle(90,12,1,10,1000);
        reset_coordinates(true,true);
        task::sleep(50);
    chassis.turn_to_angle(60);

    //Drop Goal
        chassis.drive_max_voltage = 10;
        chassis.drive_timeout = 600;
    task drop2(run);
    dist = -999;
        waitUntil(chassis.dist_travelled < -9);
        intake_speed = -50;
        Clamp.set(false);
        chassis.set_sideways(0);
        waitUntil(chassis.drive_settled);
        drop2.stop();

    intake_speed = 0;
        chassis.drive_timeout = 2000;
        chassis.drive_max_voltage = 10;

    //PickUp Last Goal
    chassis.drive_min_voltage = 4;
    chassis.drive_distance(10,45);
    arm.replace_target(state[1]);
    intake_speed = 100;
        chassis.boomerang_lead = 0.2;
        chassis.drive_max_voltage = 8;
        chassis.heading_max_voltage = 8;
    chassis.drive_to_point(-46,-22,true);
        chassis.heading_max_voltage = 12;
    chassis.turn_to_point(-26,-49,180,10,2,25,1000);
    chassis.straightline_to_point(-26,-49,false,0,6,6,2,0,1000);
    chassis.drive_min_voltage = 0;
    chassis.drive_distance(-10,chassis.get_absolute_heading(),6,6);
    Clamp.set(true);
        chassis.set_sideways(-1.55);
    task::sleep(100);
    arm.replace_target(state[1] + 40);
    chassis.turn_to_angle(90,12,1,10,1000);
    reset_coordinates(true,true);
    task::sleep(25);

    //Intake Ring to Arm
        chassis.drive_max_voltage = 10;
    chassis.turn_to_point(-43,0.5);
        chassis.drive_settle_error = 0.5;
    chassis.straightline_to_point(-43,0.5,true,0,8,10);
        chassis.drive_settle_error = 1;

    //Score Wall Stake and Ring #1
    chassis.turn_to_point(-999,0);
    intake_til_scored = true;

        chassis.drive_timeout = 700;
    chassis.drive_distance(999,chassis.get_absolute_heading(),6,0);
    arm.replace_target(state[2]);
    chassis.drive_with_voltage(1,1);
    task::sleep(400);
        chassis.drive_timeout = 2000;
        printf( "WALL2: %f \n", autonTime);

    chassis.drive_distance(-20);
    arm.replace_target(climb_pos);

    //Intake #2 and 3
    intake_til_scored = false;
    intake_speed = 100;
        chassis.drive_settle_error = 1.5;
        chassis.turn_settle_error = 5;
        chassis.turn_settle_time = 25;
    chassis.turn_to_point(-48,24);
    chassis.straightline_to_point(-48,24,true);
    //chassis.turn_to_angle(0);
    //reset_coordinates(true,true);
    //task::sleep(25);
    intake_speed = 100;
    chassis.turn_to_point(-26,24,0,6,5,10,1000);
    chassis.straightline_to_point(-26,24,true);

    //Intake #4-6
        chassis.boomerang_lead = 0.2;
        chassis.drive_min_voltage = 0;

    chassis.turn_to_point(-45,46,0,6,10,10,1000);
    intake_speed = 100;
    chassis.drive_to_point(-45,46,true,0,9,5);
        chassis.drive_min_voltage = 5;
    chassis.drive_distance(-10);
    chassis.turn_to_point(-45,61,0);
    chassis.drive_to_point(-45,61,true,0,12,12,1,10,900);
        chassis.boomerang_lead = 0;
        chassis.drive_timeout = 600;
    chassis.turn_to_angle(90,12,20,10,1000);
    chassis.drive_distance(-27,90,10,10);
    chassis.drive_timeout = 2000;
    chassis.turn_to_point(-63,54);
    chassis.straightline_to_point(-63,54,true);
        chassis.drive_settle_error = 3;
        chassis.drive_settle_time = 25;
        chassis.drive_timeout = 600;
        Arm.set(true);
    chassis.drive_distance(-13,105);
        chassis.turn_timeout = 600;
    chassis.turn_to_angle(112);

    //Drop Goal
        chassis.drive_timeout = 750;
        chassis.drive_max_voltage = 12;
        chassis.heading_max_voltage = 3;
    chassis.drive_distance(999,95);
        chassis.drive_timeout = 2000;
        chassis.heading_max_voltage = 12;
    timeDrive(100,-100,500);
        chassis.turn_timeout = 1200;
    chassis.turn_to_angle(315,12,5,25,2000);
    Arm.set(false);
        chassis.heading_max_voltage = 8;
    intake_speed = 0;
    Clamp.set(false);
        chassis.set_sideways(0);
        chassis.drive_timeout = 350;
    chassis.drive_distance(-999,315);
        chassis.drive_timeout = 2000;

    //Climb
        chassis.boomerang_lead = 0;
        chassis.drive_min_voltage = 0;
    intake_til_down = true;
    chassis.drive_to_point(-30,28,true);
    intake_til_down = false;
    intake_speed = 0;
    arm.replace_target(climb_pos);
        chassis.turn_settle_error = 1;
        chassis.turn_settle_time = 20;
    chassis.turn_to_point(0,0,180);
        chassis.drive_timeout = 1500;
        printf( "Time: %f \n", autonTime);
    chassis.drive_distance(-999,chassis.get_absolute_heading(),5,0);
    arm.replace_target(climb_pos - 45);
    chassis.drive_with_voltage(2,2);
        runTimer = false;
    printf("X: %f\nY: %f\nAngle:%f\n\n", chassis.get_X_position(), chassis.get_Y_position(),chassis.get_absolute_heading());
}

void tester(){
    TeamColor = 1;
    autonSetup();
    chassis.set_coordinates(0, 0, 90);
    task Intake_Auto(intake_auto);
    task Timer(timers);
    task Lift_Auto(lift_auto);

    arm.replace_target(state[1]);
    task::sleep(800);
    arm.replace_target(state[2]);

}