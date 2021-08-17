
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

#ifndef KALMAN_ACC_H
#define KALMAN_ACC_H

void init_kalman_acc(const position_t* initial_pos);
void kalman_prediction_step_acc(position_t* pos, double* );
void kalman_correction_step_acc(position_t* pos, double* );
void update_pos_kalman_acc(position_t* pos, double acc[2], double gps[2], bool meas_true);

#endif