/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       vex.h                                                     */
/*    Author:       Vex Robotics                                              */
/*    Created:      1 Feb 2019                                                */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//

#pragma once
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <vector>
#include <tuple>
#include <fstream>
#include <sstream>
#include <cstdlib>  
#include <stdexcept>
#include <cerrno>
#include <unistd.h>
#include <numeric>

#include "v5.h"
#include "v5_vcs.h"

#include "robot-config.h"
#include "JAR-Template/particle_filter/odom.h"
#include "JAR-Template/drive.h"
#include "JAR-Template/util.h"
#include "JAR-Template/PID.h"
#include "JAR-Template/lift.h"
#include "autons.h"
#include "paths.h"

            
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                 \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)

extern int TeamColor;