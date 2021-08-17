#include <math.h>
#include "struct.h"

#define TASK 1   // TASK: 0 is localization, 1 is flocking, 2 is formation control
#define WORLD 1  // WORLD: 0 is localization, 1 is obstacles, 2 is crossing
#define TRAJECTORY 1 // Tractectory used if task is localization

// PSO 
#define PSO true
#define ARENA_SIZE_X 5 // ARENA IS A BIT BIGGER BUT THEN ROBOT MIGHT BE PLACED OUTSIDE !
#define ARENA_SIZE_Y 3 // ARENA IS A BIT BIGGER BUT THEN ROBOT MIGHT BE PLACED OUTSIDE !
#define FIT_T 5       // Time of simulation to run for fitness during optimization

#if WORLD == 1
    #define ROBOTS_N 5
    #define TIME_IN_OBSTACLE_AVOIDANCE 2
#elif WORLD == 2
    #define ROBOTS_N 10
    #define TIME_IN_OBSTACLE_AVOIDANCE 1
#endif


/* ROBOT CONSTANTS */
#define NB_SENSORS 8
#define AXLE_LENGTH 0.052   // Distance between wheels of robot (meters)
#define WHEEL_AXIS 0.057    // Distance between the two wheels in meter
#define WHEEL_RADIUS 0.0205 // Radius of the wheel in meter
#define MAX_SPEED_WEB 6.28  // Maximum speed webots
#define ROB_RADIUS 0.035    // Robot radius

#define MAX_SPEED 800
#define BIAS_SPEED 400
#define EPS 0.0001 // Epsilon


/* VERBOSE_FLAGS */
#define VERBOSE_ENC false     	// Print odometry values computed with wheel encoders
#define VERBOSE_ACC false    	// Print odometry values computed with accelerometer
#define VERBOSE_GPS false    	// Print odometry values computed with accelerometer
#define VERBOSE_KALMAN false

#define SUPERVISOR_VERBOSE_METRIC false // print error on each timestep
#define SUPERVISOR_VERBOSE_POSITION false // print position on each timestep


/* FLOCKING */
#define FLOCK_SIZE 5

extern const position_t MIGR[];
extern const position_t INIT_POS[];

// #define COHESION_WEIGHT 0.2
// #define MIGRATION_WEIGHT 0.2
// #define DISPERSION_WEIGHT 0.4
// #define DISPERSION_THRESHOLD 0.3
// #define SPEED_MOMENTUM 0.9

#define COHESION_WEIGHT 0.15
#define MIGRATION_WEIGHT 0.15
#define DISPERSION_WEIGHT 0.4
#define DISPERSION_THRESHOLD 0.2
#define SPEED_MOMENTUM 0.8

#define D_FLOCK DISPERSION_THRESHOLD    // TO DEFINE: targeted flocking distance
#define D_MAX_FLOCK 0.1 // TO DEFINE: maximum distance travelled in 1 timestep: timestep * maximum speed


/* FORMATION */
#define LEADER_ID 0
#define D_MAX_FORM 2 // TO DEFINE: maximum distance travelled in 1 timestep: timestep * maximum speed


/* MATH HELPER */
#define sign(X) (2*(X >= 0) - 1)
