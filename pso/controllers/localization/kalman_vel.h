
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <webots/robot.h>
#include <webots/motor.h>

#include <gsl/gsl_blas.h>

#include "../const.h"
#include "../struct.h"
#include "gsl_helper.h"
#include "kalman_variable.h"

#ifndef KALMAN_VEL_H
#define KALMAN_VEL_H

void init_kalman_vel(const position_t* initial_pos);
void kalman_prediction_step_vel(position_t* pos, double* );
void kalman_correction_step_vel(position_t* pos, double* );
void update_pos_kalman_vel(position_t* pos, double vel[2], double gps[2], bool meas_true);

#endif