#include <stdbool.h>

#ifndef STRUCT_H
#define STRUCT_H 

typedef struct 
{
    double x;
    double y;
    double heading;
} position_t;

typedef struct 
{
    double prev_gps[3];
    double gps[3];
    double acc_mean[3];
    double acc[3];
    double prev_left_enc;
    double left_enc;
    double prev_right_enc;
    double right_enc;
    bool gps_true;
} measurement_t;

#endif