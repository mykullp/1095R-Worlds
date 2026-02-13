#pragma once
#include "JAR-Template/drive.h"
#pragma once

//extern void [name];
class Drive;
class Lift;

extern Drive chassis;
extern Lift arm;

void default_constants();
extern void timeDrive(double lspeed, double rspeed, double time);
extern double autonTime;
extern bool auto_started;
extern bool colorsort_on;
extern bool antijam_on;

extern void anti_jam();
extern void color_sort();

extern void autonSetup();

extern void tester();
extern void skills();

extern void red_solo_six_wp();
extern void blue_solo_six_wp();

extern void red_solo_wp_disrupt();
extern void blue_solo_wp_disrupt();

extern void red_goal_rush();
extern void blue_goal_rush();

extern void red_solo_seven_wp();
extern void blue_solo_seven_wp();

extern void red_pos_wp();
extern void blue_pos_wp();

extern void red_goal_rush_wall_disrupt();
extern void blue_goal_rush_wall_disrupt();

extern bool runTimer;
extern bool intake_til_hold;
extern void reset_coordinates(bool V, bool H);