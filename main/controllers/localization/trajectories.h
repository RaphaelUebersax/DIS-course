#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H 

#include <webots/robot.h>
#include <webots/motor.h>

// ## DO NOT MODIFY THIS
bool trajectory_1(WbDeviceTag dev_left_motor, WbDeviceTag dev_right_motor, double time_end_calibration);
bool trajectory_2(WbDeviceTag dev_left_motor, WbDeviceTag dev_right_motor, double time_end_calibration);

// ## but you can add your own trajectories if you like.

#endif
