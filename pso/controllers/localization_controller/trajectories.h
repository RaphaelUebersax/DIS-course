#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H 

#include <webots/robot.h>
#include <webots/motor.h>

// ## DO NOT MODIFY THIS
void trajectory_1(WbDeviceTag dev_left_motor, WbDeviceTag dev_right_motor, bool acc_cal);
void trajectory_2(WbDeviceTag dev_left_motor, WbDeviceTag dev_right_motor, bool acc_cal);

// ## but you can add your own trajectories if you like.

#endif