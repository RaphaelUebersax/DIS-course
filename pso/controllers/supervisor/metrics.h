#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <webots/robot.h>

#include "../const.h"


int update_localization_metric(float loc_abs[ROBOTS_N][3], float loc_est[ROBOTS_N][3]);
int update_flocking_metric(float loc_abs[ROBOTS_N][3]);
int update_formation_metric(float loc_abs[ROBOTS_N][3]);
double get_metric();
void reset_metric();
void save_metric(wchar_t* name, double metric);
float dist(float v1[2], float v2[2]);
