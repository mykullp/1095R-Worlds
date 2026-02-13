#pragma once
#include "vex.h"

class Odom
{
private:
  float ForwardTracker_center_distance;
  float SidewaysTracker_center_distance;
  float ForwardTracker_position;
  float SideWaysTracker_position;
public:
  float X_position;
  float Y_position;
  float vertical_offset;
  float horizontal_offset;
  float orientation_deg;
  void set_position(float X_position, float Y_position, float orientation_deg, float ForwardTracker_position, float SidewaysTracker_position);
  void update_position(float ForwardTracker_position, float SidewaysTracker_position, float orientation_deg);
  void set_physical_distances(float ForwardTracker_center_distance, float SidewaysTracker_center_distance);
};

extern double vertical_dist();
extern double horizontal_dist();
