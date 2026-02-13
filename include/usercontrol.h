
#pragma once
#include "vex.h"

extern std::vector<double> state;

extern int user_cont();
extern int lift_macro();
extern int pneumatics_toggle();

extern int count;
extern double alliance_pos;
extern double climb_pos;

extern bool clampDown;
extern bool armDown;
extern bool clawDown;
extern bool climb_toggle;
extern bool tipper_out;

extern bool lift_move;

extern bool primed;
extern bool colorSort;
