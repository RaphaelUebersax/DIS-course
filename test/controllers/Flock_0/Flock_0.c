/*****************************************************************************/
/* File:         reynolds.c                                                 */
/* Version:      3.0                                                         */
/* Date:         6-June-21                                                   */
/* Description:  Flocking and formation algorithms for multi-robot systems	*/
/*               using range and bearing from realtive positioning           */
/* Authors:      DIS group 12		                                             */
/*****************************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define max 20    

#include "../hungarian.h"

#include <webots/robot.h>
#include <webots/motor.h>

#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>

typedef struct 
{
  double x;
  double y;
  double heading;
} pose_t;

#include "utils.h"




//--------------------------------------------------------------------------------------------------------------//
//---------------------------------- VARIABLES TO SET ----------------------------------------------------------//

/* ALGROITHM TO RUN */
#define FLOCKING false       // TODO: SET TO TRUE IF TEST FLOCKING ALGORITHM
#define FORMATION true       // TODO: SET TO TRUE IF TEST FORMATION ALGORITHM

/* LOCALIZATION CAPABILITIES OF THE ROBOTS */
#define ENCODER_LOC false    // TODO: SET TO TRUE IF LOCALIZATION BASED ON ENCODERS
#define ODOMETRY_ACC false   // TODO: SET TO TRUE IF LOCALIZATION BASED ON ACCELEROMETER
#define GPS_LOC false         // TODO: SET TO TRUE IF LOCALIZATION BASED ON GPS AND ODOMETRY
#define ACTIVATE_KALMAN true // TDOO: SET TO TRUE IF LOCALIZATION BASED ON KALMAN FILTER 


/* FLOCK */
#define FLOCK_SIZE 5           // TODO: SET THE RIGHT NUMBER OF ROBOTS IN ONE FLOCK
#define FLOCK_NUMBER 0         // TODO: SET THE FLOCK NUMBER THE SAME FOR ALL ROBOTS THAT ARE IN THE SAME FLOCK
#define WORLD 0                // TODO: SET WORLD TO {0: OBSTACLES, 1: CROSSING, 2: OBSTACLES_7_ROBOTS, 3: OBSTACLES_9_ROBOTS, ELSE: FREE}

// TODO: DECIDE THE LAPLACIAN FORMATION                         
double initial_formation[2][FLOCK_SIZE] = {{0,0,0,-0.2,0.2},{0.2,-0.2,0,0,0}};                                     // DESIRED FORMATION !!! (5 robots)
//double initial_formation[2][FLOCK_SIZE] = {{0,0,0,-0.2,0.2,-0.2,0.2},{0.3,-0.3,0,0.2,0.2,-0.2,-0.2}};              // DESIRED FORMATION !!! (7 robots)
//double initial_formation[2][FLOCK_SIZE] = {{0,0,0,-0.2,0.2,-0.2,0.2,0.5,0.3},{0.4,-0.4,0,0.2,0.2,-0.2,-0.2,0,0}}; // DESIRED FORMATION !!! (9 robots)

// TODO: DECIDE THE MIGRATORY URGE
float migr[2] = {5.0, 0.0};                                                                 // MIGRATION VECTOR !!!
//float migr[2] = {5.0, 1.0}; 
//float migr[2] = {5.0, -1.0}; 
//float migr[2] = {5.0, 2.0}; 
//float migr[2] = {5.0, -2.0}; 


//-------------------------------------------------------------------------------------------------------------//




// VERBOSE to print  intermediary results
#define VERBOSE_GPS false
#define VERBOSE_ENC false
#define VERBOSE_ACC false
#define VERBOSE_POS false
#define VERBOSE_ACC_MEAN false
#define VERBOSE_KF false


/*CONSTANTS*/
#define WHEEL_AXIS  0.057     // Distance between the two wheels in meter
#define TIME_INIT_ACC 5                    // Time in second


// Formation related
#define NB_SENSORS                8 // Number of distance sensors
#define MIN_SENS                    350     // Minimum sensibility value
#define MAX_SENS                    4096    // Maximum sensibility value
#define MAX_SPEED                   800     // Maximum speed
/*Webots 2018b*/
#define MAX_SPEED_WEB               3.0    // Maximum speed for our robot
/*Webots 2018b*/
#define TIME_STEP               64  // [ms] Length of time step

#define AXLE_LENGTH                 0.052   // Distance between wheels of robot (meters)
#define WHEEL_RADIUS                0.0205  // Wheel radius (meters)
#define DELTA_T               0.064 // Timestep (seconds)

#define K_cohesion                  0.35     // Weight of aggregation rule.

#define RULE2_THRESHOLD             0.1    // Threshold to activate dispersion rule. 
#define K_separation                0.04  // Weight of dispersion rule.


#define K_migration                 0.28    // Wheight of attraction towards the common goal.
#define K_migration_laplacian       0.2   // Migration urge for the formation controller

#define K_laplacian                 1.05   // weight of the Laplacian with respect to migratory urge 
#define TARGET_THRESHOLD            0.0    // Distance from the taget at which it is considered as correct

#define ABS(x) ((x>=0)?(x):-(x))



//-------------------------------------------------------------------------------------------------//
//----------------------------- Definition of structures ------------------------------------------//
//-------------------------------------------------------------------------------------------------//

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

} measurement_t;



typedef struct
{
  float supervisor[3];
  pose_t pos;
  pose_t speed;
  pose_t acc;
  int id;
} robot_t;




//-------------------------------------------------------------------------------------------------//
//--------------------- DEVICE TAGS FOR LOCALISTION AND COMMUNICATION -----------------------------//
//-------------------------------------------------------------------------------------------------//

// Handle for localisation devices
WbDeviceTag dev_gps;
WbDeviceTag dev_acc;
WbDeviceTag dev_left_encoder;
WbDeviceTag dev_right_encoder;

// Handle for motor to set velocities
WbDeviceTag dev_left_motor;
WbDeviceTag dev_right_motor;

// TO REMOVE !!!!
WbDeviceTag actual_pos;

WbDeviceTag ds[NB_SENSORS];	// Handle for the infrared distance sensors
WbDeviceTag receiver2;		// Handle for the receiver node
WbDeviceTag emitter2;		// Handle for the emitter node





//-------------------------------------------------------------------------------------------------//
//----------------------------- GLOBAL VARIABLES --------------------------------------------------//
//-------------------------------------------------------------------------------------------------//



int cost_matrix [FLOCK_SIZE][FLOCK_SIZE];   // Cost matrix ufor Laplacian assignment
int local_urge = 1;                         // Local migration urge (true or false)
int robot_id_u, robot_id;	                  // Unique and normalized (between 0 and FLOCK_SIZE-1) robot ID
int robot_assignment;                       // Assignment of the robot in Laplacian controller
int assignment_vect[FLOCK_SIZE];            // Assugnement vector for all robots in Laplacian controller

float relative_pos[FLOCK_SIZE][3] ;	        // relative X, Z, Theta of all robots
float prev_relative_pos[FLOCK_SIZE][3];	    // Previous relative  X, Z, Theta values
float my_position[3]= {0,0,0};     		      // X, Z, Theta of the current robot
float prev_my_position[3];  		            // X, Z, Theta of the current robot in the previous time step
float speed[FLOCK_SIZE][2];		              // Speeds calculated with FLOCKING/FORMATION's rules
float relative_speed[FLOCK_SIZE][2];	      // Speeds calculated with FLOCKING/FORMATION's rules
int initialized[FLOCK_SIZE];		            // != 0 if initial positions have been received
char* robot_name;                           // Name of the robot
float theta_robots[FLOCK_SIZE];             // Orientations of each robot 



double obstacles_table[2][NB_SENSORS];      // Sensor value for each sensor
double distances_after_transf[NB_SENSORS];  // Distance corresponding on sensor value
double sensor_orientations[NB_SENSORS] = {-0.30 , -0.80, -1.57, -2.64, 2.64, 1.57, 0.80, 0.30 };  // Orientation of each sensor of e-puck
double min_range_detect = 0.04  ;           // Threshold for considering obstacle detection

double target_pose_x;     // Target position x of the robot in Laplacian controller
double target_pose_y;     // Target position y of the robot in Laplacian controller


double msl=0 ;      //
double msr=0 ;			//   Wheel speeds
double msl_w = 0;   //
double msr_w = 0;   //

int i;				      // Loop counter
double distances[NB_SENSORS];	// Array for the distance sensor readings


// Covarinace matrix used for Kalman Filter
static double KF_cov[MMS][MMS] = {{0.001, 0, 0, 0},
                                  {0, 0.001, 0, 0},
                                  {0, 0, 0.001, 0},
                                  {0, 0, 0, 0.001}};

double last_gps_time_s = 0.0f;    // Time of last GPS measurement
double time_end_calibration = 0;  // Time of the end of calibration 


int state = 0; 
int prev_state = 0;
int obst = 0; //zero for no obstacle 

int transition_time = 0; 
int max_transition_time = 15; 





//-----------------------------------------------------------------------------------//
//------------------------- DEFINE STURCT VARIABLES ---------------------------------//

static measurement_t  _meas;
static robot_t        _robot;






//-----------------------------------------------------------------------------------//
// ------------------------ FUNCTION PROTOTYPES -------------------------------------//

static void reset();
static void controller_get_acc();
static void controller_get_encoder();
static void controller_get_gps();
static void controller_compute_mean_acc();

void init_devices(int ts);
void KF_Update_Cov_Matrix(double ts);
void Kalman_Filter();
int** array_to_matrix(int* m, int rows, int cols) ;
void laplacian();
double map_sensor_to_dist(double sensor_val);
void update_obstacle_range_bearing();
void limit(int *number, int limit);
void update_self_motion(double msl, double msr);
void OA();
void compute_wheel_speeds();
void reynolds_rules();
void send_laplacian_target_to_supervisor(void);
void send_ping(void);
void process_received_ping_messages(void);
void update_cost_matrix();
void initial_position(int robot_nb);







//-----------------------------------------------------------------------------------//
//--------------------------- MAIN FUNCTION -----------------------------------------//
/*
  The main computes the localization of the robots before updating the range and bearing
  measurements for obtaining the relative position between the flock of robots. Depending
  on the choice of algorithm (formation/flocking), it then computes the wheel speeds using
  the update rules as well as obstacle avoidance. 
*/
//-----------------------------------------------------------------------------------//

int main(){ 

  // Verifiy that one method only is selected
  if ((FLOCKING && FORMATION) || ((!FLOCKING) && (!FORMATION))){
    printf("INVALID CHOICE OF FORMATION/FLOCKING\n");
    return 1;
  }
  

  wb_robot_init(); // Initialize webots variabes
  reset();         // Resetting the robot

  int time_step = wb_robot_get_basic_time_step(); // Time step of the simulator
  init_devices(time_step);                        // Initialize the different devices

  msl = 0; msr = 0;       // Set wheel speeds to zero
  hungarian_problem_t p;  // Initialize variable for the Hungarian assignment problem
  
  int iter = 0;           // initialize iteration to 0

  initial_position(robot_id);     // initialize position

  // Assign starting speed to 0
  _robot.speed.x = 0;
  _robot.speed.y = 0;

  // Make sure axis are good
  if (WORLD == 1){
    migr[0] = (-1)*migr[0];
    _robot.pos.y = (-1)*_robot.pos.y;
  }
          
  // Forever
  while (wb_robot_step(time_step) != -1)  {
                              
    /* Send and get information */
    send_ping();  // sending a ping to other robot, so they can measure their distance to this robot
    
    // Not on first iteration !
    if (iter!=0) {


      if (GPS_LOC){
      
        double time_now_s = wb_robot_get_time();
        
        if (time_now_s - last_gps_time_s >= 1.0f) {
          
          last_gps_time_s = floor(time_now_s);
          controller_get_gps();
           _robot.pos.x = _meas.gps[0];
           _robot.pos.y = _meas.gps[2];


            ///////HEADING///////// Uses encoder for heading as GPS does not provide this information !!!!!!
            _meas.prev_left_enc = _meas.left_enc;
            _meas.left_enc = wb_position_sensor_get_value(dev_left_encoder);
            double deltaleft = _meas.left_enc-_meas.prev_left_enc;
            _meas.prev_right_enc = _meas.right_enc;
            _meas.right_enc = wb_position_sensor_get_value(dev_right_encoder);
            double deltaright = _meas.right_enc-_meas.prev_right_enc;
            deltaleft  *= WHEEL_RADIUS;
            deltaright *= WHEEL_RADIUS;
            double omega = -( deltaright - deltaleft ) / ( WHEEL_AXIS * time_step); //ADDED MINUS TO TEST
            _robot.pos.heading += omega * time_step; 
          }
      }
      // Get localization depending on choice of method
      else if(ODOMETRY_ACC) {
         if(wb_robot_get_time() < TIME_INIT_ACC) {
           // Wait not to bias the accelerometers.
           wb_motor_set_velocity(dev_left_motor, 0);
           wb_motor_set_velocity(dev_right_motor, 0);
    
           controller_compute_mean_acc();
             
            //time_end_calibration = wb_robot_get_time(); // not used?
            continue; 
          }
          else {
            controller_get_acc();
          }
      }
      else {
        controller_get_encoder();  
      }

      if (VERBOSE_POS)
        printf("ROBOT %d, pose: %g %g %g\n", 
          robot_id, _robot.pos.x , _robot.pos.y, _robot.pos.heading);

      // Update the covariance matrix for Kalman filter.
      if (ACTIVATE_KALMAN) {
      
        KF_Update_Cov_Matrix((double) time_step/1000);
        double time_now_s = wb_robot_get_time();
        
        if (time_now_s - last_gps_time_s >= 1.0f) {
          
          last_gps_time_s = floor(time_now_s);
          controller_get_gps();
    
          //printf("ACC1: %g %g %g\n", _robot.acc.x , _robot.acc.y, _robot.acc.heading);
          Kalman_Filter();
          //printf("ACC2: %g %g %g\n", _robot.acc.x , _robot.acc.y, _robot.acc.heading);
    
          if (VERBOSE_POS){
            printf("ROBOT %d, pose after Kalman: %g %g %g\n\n",
              robot_id, _robot.pos.x , _robot.pos.y, _robot.pos.heading);
          }
        }
      }   

      // Assign position of the robot depending on results of localization techniques and axes definitions
      my_position[0] = _robot.pos.x;
      my_position[1] = (-1.0)*_robot.pos.y;
      my_position[2] = (-1.0)*_robot.pos.heading;

    }

    // Set iter to 1 to execute localization from now on
    iter = 1;

    // Store previous position
    prev_my_position[0] = my_position[0];
    prev_my_position[1] = my_position[1];


    if (FORMATION){

      update_cost_matrix(); 
      int *r = (int*)cost_matrix;
      int** m = array_to_matrix(r,FLOCK_SIZE,FLOCK_SIZE);

      /* initialize the gungarian_problem using the cost matrix*/
      hungarian_init(&p, m , FLOCK_SIZE,FLOCK_SIZE, HUNGARIAN_MODE_MINIMIZE_COST) ;

      /* solve the assignement problem */
      hungarian_solve(&p);

      for (int j = 0; j<FLOCK_SIZE; j++){
          if (*(p.assignment[robot_id]+j) == 1 ){
              //save the new assignment j of robot i 
              robot_assignment = j; 
          }     
        }
      
      for (int i = 0; i<FLOCK_SIZE; i++){
        //printf("row 1 : %d %d %d %d %d\n", cost_matrix[i][0], cost_matrix[i][1],cost_matrix[i][2],cost_matrix[i][3],cost_matrix[i][4]);  
      }
        
        /* free used memory */
        hungarian_free(&p);

        int idx;
        for (idx=0; idx < FLOCK_SIZE; idx+=1) {
          free(m[idx]);
        }
        free(m);
      }

    
    
    // Update range an bearing based on receiver
    process_received_ping_messages();


    
    // Reynold's rules with all previous info (updates the speed[][] table)
    if (FLOCKING){
        reynolds_rules();
    }
    //Laplacian controller with all previous info (updates the speed[][] table)
    if (FORMATION){
        laplacian();
    }

    
    // Obstacle avoidance
    OA(); 
    
       if (obst){
       state = 1;
        transition_time = 0;
        }
      //if no obstcale is detcted, check if the previous state was navigation in formation  
      //if so, keep the state 0 and reset the transition timer  
      else if (prev_state == 0){
          state = 0; 
          transition_time = 0;}


      //if prev state is OA or transition and the current state is neither OA nor navigation in formation, 
      //check if go/stay in transition or if go back to state 0  
        else {

      //if in the transition state and in the time margin: stay in transition phase and update timer 
      if (transition_time < max_transition_time){
        state = 2; 
        transition_time = transition_time + 1; 
      //if spent more than max time in the transition state, switch back to formation state 
      }else {

        state = 0; 
        transition_time = 0;
      }
    
    }
      //update the state for next iteration 
      prev_state = state; 

    // Compute wheels speed from reynold's speed
    compute_wheel_speeds();
    
    // Tell the speed to the motors
    wb_motor_set_velocity(dev_left_motor, msl);
    wb_motor_set_velocity(dev_right_motor, msr);
                      
    // Update position based on odometry for time steps where localization is not available               
    // // Not used? Overriden by the odometry/Kalman etc.?
    //update_self_motion(msl, msr);
  }



  // End of the simulation
  wb_robot_cleanup();
  return 0;
}














//------------------------------------------------------------------------------------------------------//
// ------------------------------- INITIALIZE ALL WEBOTS DEVICES ---------------------------------------//
/*
  This function initializes all webots specific devices. In this case it initializes the GPS, accelerometer,
  left and right encoders as well as left and right mototrs. Note that receiver and emitter are initialzed
  in the reset function rather than here.
*/
//------------------------------------------------------------------------------------------------------//

void init_devices(int ts) {

  // Initialize GPS
  dev_gps = wb_robot_get_device("gps");
  wb_gps_enable(dev_gps, 1000);

  // Initialze accelerometer
  dev_acc = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(dev_acc, ts);

  // Initialize encoders
  dev_left_encoder = wb_robot_get_device("left wheel sensor");
  dev_right_encoder = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(dev_left_encoder, ts);
  wb_position_sensor_enable(dev_right_encoder, ts);

  // Initialize motors
  dev_left_motor = wb_robot_get_device("left wheel motor");
  dev_right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(dev_left_motor, INFINITY);
  wb_motor_set_position(dev_right_motor, INFINITY);
  wb_motor_set_velocity(dev_left_motor, 0.0);
  wb_motor_set_velocity(dev_right_motor, 0.0);
}







//------------------------------------------------------------------------------------------------------//
// ------------------------------- INITIALIZE POSITION OF ROBOT ----------------------------------------//
/*
  This function initializes the position of the robot based on the world. If world ==0, the world is
  "obstacles.wbt" and if WORLD == 1 the world is "crossing.wbt". Else  make sure to change the inital
  position to what is desired.
*/
//------------------------------------------------------------------------------------------------------//

void initial_position(int robot_nb){

  // OBSTACLES.WBT
  if (WORLD == 0){
    double heading = 0; //M_PI / 2;

    switch (robot_nb){

      case 0:
        _robot.pos.x = -2.9;
        _robot.pos.y = 0;
        _robot.pos.heading = heading;
        break;

      case 1:
        _robot.pos.x = -2.9;
        _robot.pos.y = 0.1;
        _robot.pos.heading = heading;
        break;

      case 2:
        _robot.pos.x = -2.9;
        _robot.pos.y = -0.1;
        _robot.pos.heading = heading;
        break;

      case 3:
        _robot.pos.x = -2.9;
        _robot.pos.y = 0.2;
        _robot.pos.heading = heading;
        break;

      case 4:
        _robot.pos.x = -2.9;
        _robot.pos.y = -0.2;
        _robot.pos.heading = heading; 
        break;

    }

  }

  // CROSSING.WBT
  else if (WORLD == 1){
  
    double heading = M_PI;
    switch (robot_nb){

      case 0:
        _robot.pos.x = -0.1;
        _robot.pos.y = 0;
        _robot.pos.heading = heading;
        break;

      case 1:
        _robot.pos.x = -0.1;
        _robot.pos.y = -0.1;
        _robot.pos.heading = heading;
        break;

      case 2:
        _robot.pos.x = -0.1;
        _robot.pos.y = 0.1;
        _robot.pos.heading = heading;
        break;

      case 3:
        _robot.pos.x = -0.1;
        _robot.pos.y = -0.2;
        _robot.pos.heading = heading;
        break;

      case 4:
        _robot.pos.x = -0.1;
        _robot.pos.y = 0.2;
        _robot.pos.heading = heading;
        break;

    }


  }

    // OBSTACLES_7_robots.WBT
  else if (WORLD == 2){
    double heading = 0; //M_PI / 2;

    switch (robot_nb){

      case 0:
        _robot.pos.x = -2.9;
        _robot.pos.y = 0;
        _robot.pos.heading = heading;
        break;

      case 1:
        _robot.pos.x = -2.9;
        _robot.pos.y = 0.1;
        _robot.pos.heading = heading;
        break;

      case 2:
        _robot.pos.x = -2.9;
        _robot.pos.y = -0.1;
        _robot.pos.heading = heading;
        break;

      case 3:
        _robot.pos.x = -2.9;
        _robot.pos.y = 0.2;
        _robot.pos.heading = heading;
        break;

      case 4:
        _robot.pos.x = -2.9;
        _robot.pos.y = -0.2;
        _robot.pos.heading = heading; 
        break;

      case 5:
        _robot.pos.x = -2.9;
        _robot.pos.y = -0.3;
        _robot.pos.heading = heading;
        break;

      case 6:
        _robot.pos.x = -2.9;
        _robot.pos.y = 0.3;
        _robot.pos.heading = heading; 
        break;



    }

  }

    else if (WORLD == 3){
    double heading = 0; //M_PI / 2;

    switch (robot_nb){

      case 0:
        _robot.pos.x = -2.9;
        _robot.pos.y = 0;
        _robot.pos.heading = heading;
        break;

      case 1:
        _robot.pos.x = -2.9;
        _robot.pos.y = 0.1;
        _robot.pos.heading = heading;
        break;

      case 2:
        _robot.pos.x = -2.9;
        _robot.pos.y = -0.1;
        _robot.pos.heading = heading;
        break;

      case 3:
        _robot.pos.x = -2.9;
        _robot.pos.y = 0.2;
        _robot.pos.heading = heading;
        break;

      case 4:
        _robot.pos.x = -2.9;
        _robot.pos.y = -0.2;
        _robot.pos.heading = heading; 
        break;

      case 5:
        _robot.pos.x = -2.9;
        _robot.pos.y = -0.3;
        _robot.pos.heading = heading;
        break;

      case 6:
        _robot.pos.x = -2.9;
        _robot.pos.y = 0.3;
        _robot.pos.heading = heading; 
        break;

      case 7:
        _robot.pos.x = -2.9;
        _robot.pos.y = -0.4;
        _robot.pos.heading = heading;
        break;

      case 8:
        _robot.pos.x = -2.9;
        _robot.pos.y = 0.4;
        _robot.pos.heading = heading; 
        break;



    }

  }

  // FREE TO CHANGE
  else{

    switch (robot_nb){

      case 0:
        _robot.pos.x = 0;
        _robot.pos.y = 0;
        _robot.pos.heading = 0;
        break;

      case 1:
        _robot.pos.x = 0;
        _robot.pos.y = 0;
        _robot.pos.heading = 0;
        break;

      case 2:
        _robot.pos.x = 0;
        _robot.pos.y = 0;
        _robot.pos.heading = 0;
        break;

      case 3:
        _robot.pos.x = 0;
        _robot.pos.y = 0;
        _robot.pos.heading = 0;
        break;

      case 4:
        _robot.pos.x = 0;
        _robot.pos.y = 0;
        _robot.pos.heading = 0;
        break;

    }

  }
}












//-------------------------------------------------------------------------------------------------------//
// ------------------------ UPDATE COVARIANCE MATRIX OF KALMAN FILTER -----------------------------------//
/*
  This function update the covariance matrix of the Kalman filter 
*/
//-------------------------------------------------------------------------------------------------------//

void KF_Update_Cov_Matrix(double ts){
  double A[MMS][MMS]={{1, 0, ts, 0},
                      {0, 1, 0, ts},
                      {0, 0, 1, 0 },
                      {0, 0, 0, 1 }};

  double R[MMS][MMS]={{0.05, 0, 0, 0},
                      {0, 0.05, 0, 0},
                      {0, 0, 0.01, 0},
                      {0, 0, 0, 0.01}};

  double A_cov[MMS][MMS];
  double AT[MMS][MMS];
  double A_cov_AT[MMS][MMS];
  double ts_R[MMS][MMS];

  mult(A,KF_cov,A_cov,4,4,4,4);
  transp(A,AT,4,4);
  mult(A_cov, AT, A_cov_AT, 4,4,4,4);

  scalar_mult(ts, R, ts_R, 4, 4);
  add(A_cov_AT, ts_R, KF_cov, 4,4,4,4);
  
}



//-----------------------------------------------------------------------------------------------------//
// ----------------------- COMPUTE POSITION USING KALMAN FILTER ---------------------------------------//
/*
  This function uses the a Kalman filter with the encoder and GPS measurement to infer the robot's 
  localization. The computed position and orientation are then stored in the "_robot" variable
*/
//-----------------------------------------------------------------------------------------------------//

void Kalman_Filter(){
  static double X[MMS][MMS];
  X[0][0]=_robot.pos.x;
  X[1][0]=_robot.pos.y;
  X[2][0]=_robot.speed.x;
  X[3][0]=_robot.speed.y;
  
  if (VERBOSE_KF){
    printf("______________________________________________\n");
    printf("Before\n");
    printf("Cov matrix\n");
    print_matrix(KF_cov, 4,4);
    printf("X matrix\n");
    print_matrix(X, 4,1);
  }

  static double C[MMS][MMS]={{1, 0, 0, 0},
                             {0, 1, 0, 0}};
  static double Q[MMS][MMS]={{1, 0},{0, 1}};

  static double Z[MMS][MMS];
  Z[0][0] = _meas.gps[0];
  Z[1][0] = _meas.gps[2];

  static double X_new[MMS][MMS];

  static double K[MMS][MMS];
  static double temp1[MMS][MMS];
  static double temp2[MMS][MMS];
  static double cov_Ct[MMS][MMS];
  static double eye4[MMS][MMS]={{1, 0, 0, 0},{0, 1, 0, 0},{0, 0, 1, 0},{0, 0, 0, 1}};

  transp(C, temp1, 2, 4);
  mult(KF_cov,temp1,cov_Ct,4,4,4,2);
  mult(C,cov_Ct,temp1,2,4,4,2); 
  add(temp1, Q, temp2, 2,2,2,2);
  inv(temp2, temp1);
  mult(cov_Ct,temp1,K,4,2,2,2);

  mult(C, X, temp2, 2,4,4,1);
  scalar_mult(-1, temp2, temp1, 2,1);
  add(Z, temp1, temp2, 2,1,2,1);
  mult(K, temp2, temp1, 4,2,2,1);
  add(X, temp1, X_new, 4,1,4,1);
   
  mult(K,C,temp1, 4,2,2,4);
  scalar_mult(-1, temp1, temp2, 4,4);
  add(eye4, temp2, temp1, 4,4,4,4);
  mult(KF_cov, temp1, temp2, 4,4,4,4);
  copy_matrix(temp2, KF_cov, 4,4);
    
  _robot.pos.x   = X_new[0][0];
  _robot.pos.y   = X_new[1][0];
  _robot.speed.x = X_new[2][0];
  _robot.speed.y = X_new[3][0];


  if (VERBOSE_KF){
    printf("After\n");
    printf("Cov matrix\n");
    print_matrix(KF_cov, 4,4);
  
    printf("X matrix\n");
    print_matrix(X_new, 4,1);
  }
}








//------------------------------------------------------------------------------------------------------//
//----------------------------- CONVERTS ARRAY TO MATRIX -----------------------------------------------//
/*
  This helper function is used to convert an referenced array into a readable matrix
*/
//-----------------------------------------------------------------------------------------------------//

int** array_to_matrix(int* m, int rows, int cols) {
  int i,j;
  int** r;
  r = (int**)calloc(rows,sizeof(int*));
  for(i=0;i<rows;i++)
  {
    r[i] = (int*)calloc(cols,sizeof(int));
    for(j=0;j<cols;j++)
      r[i][j] = m[i*cols+j];
  }
  return r;
}




//----------------------------------------------------------------------------------------------------//
//---------------------------- COMPUTE LAPLACIAN FORMATION RULES -------------------------------------//
/*
  This function computes the rules or the formation algorithm. It is based on a Laplacian controller that
  computes the wheel speed such that the robot follow the initially defined fromation. Additionally, a
  migratory urge is added to ensure that the group of robot follows a common goal.
*/
//----------------------------------------------------------------------------------------------------//

void laplacian() {


    double temp_formation[2][FLOCK_SIZE]; // Variable that stores a temporary formation

    double form_lapl_consensu_x = 0.0 ;   // 
    double form_lapl_consensu_y = 0.0;    // Initialize the consensus to 0
    double robots_lapl_consensu_x = 0.0 ; //
    double robots_lapl_consensu_y = 0.0 ; //

     
    for (int i = 0; i<FLOCK_SIZE; i++){

        // Express the formation from the robot's center
        temp_formation[0][i] = initial_formation[0][i] - initial_formation[0][robot_assignment];
        temp_formation[1][i] = initial_formation[1][i] - initial_formation[1][robot_assignment];

        // Compute the Laplacian consensus of the formation
        form_lapl_consensu_x = form_lapl_consensu_x + 1./FLOCK_SIZE * temp_formation[0][i];
        form_lapl_consensu_y = form_lapl_consensu_y + 1./FLOCK_SIZE * temp_formation[1][i];

        // Compute the Laplacian consensus of the robots
        robots_lapl_consensu_x = robots_lapl_consensu_x + 1./FLOCK_SIZE * relative_pos[i][0];
        robots_lapl_consensu_y = robots_lapl_consensu_y + 1./FLOCK_SIZE * relative_pos[i][1];

    }

    // Compute the formation bias
    double form_bias_x = temp_formation[0][robot_assignment] - form_lapl_consensu_x;
    double form_bias_y = temp_formation[1][robot_assignment] - form_lapl_consensu_y;

    // Express the robot's formation in its frame 
    double local_form_bias_x = form_bias_x*cosf(my_position[2]) + form_bias_y*sinf(my_position[2]); 
    double local_form_bias_y = -form_bias_x*sinf(my_position[2]) + form_bias_y*cosf(my_position[2]); 


    // Define the target position of the robot
    target_pose_x = robots_lapl_consensu_x + local_form_bias_x;
    target_pose_y = robots_lapl_consensu_y + local_form_bias_y;

    // Set the required precision of the robot to its target
    if (target_pose_x < TARGET_THRESHOLD){
        target_pose_x = 0;
        }
    if (target_pose_y < TARGET_THRESHOLD){
        target_pose_y = 0;
        }
    
    // Compute migratory urge direction
    double dist_to_goal_x = (migr[0]-my_position[0]);
	  double dist_to_goal_y = (migr[1]-my_position[1]);
	
    // Express the distance to goal in robot frame
	  double local_dist_to_goal_x = dist_to_goal_x*cosf(my_position[2]) + dist_to_goal_y*sinf(my_position[2]); 
    double local_dist_to_goal_y = -dist_to_goal_x*sinf(my_position[2]) + dist_to_goal_y*cosf(my_position[2]); 

    // Compute the distance to the goal (norm of x and y)
	  double range_to_goal = sqrt(local_dist_to_goal_x* local_dist_to_goal_x + local_dist_to_goal_y * local_dist_to_goal_y);

    // If the robot is close to goal, use a proportional weight to the distance ot goal
  	if(range_to_goal < 0.5){
  	

    	     speed[robot_id][0] = dist_to_goal_x * K_migration + target_pose_x * K_laplacian;
          	     speed[robot_id][1] = dist_to_goal_y * K_migration + target_pose_y * K_laplacian; 

  	}
    // If far from goal, take a normalized version of the migratory urge
    else{

      	     speed[robot_id][0] = local_dist_to_goal_x / range_to_goal * K_migration_laplacian + target_pose_x * K_laplacian;
      	     speed[robot_id][1] = local_dist_to_goal_y / range_to_goal * K_migration_laplacian + target_pose_y * K_laplacian;
  	}
	

}





//------------------------------------------------------------------------------------------------------//
//----------------------------- MAP SENSOR VALUE TO CORRESPONDING DISTANCES ----------------------------//
/*
  This function maps the infrared sensor values to the corresponding distance using linear interpolation
  and based on the values provided by the Webots documentation of the e-puck
*/
//------------------------------------------------------------------------------------------------------//

double map_sensor_to_dist(double sensor_val){

    double sensor_mapp[6] = {4095,3474, 2211,676,306,34};
    double dist_mapp[6] = {0.0,0.005,0.01,0.02,0.03,0.07};

    int a_index, b_index;
    double x_a, x_b;
    double y_a, y_b;
    double distance;

    for (int i=0; i!=5; i++){
        if (sensor_val >= sensor_mapp[0]){
            distance = 0.0;
            break;
        }
        else if (sensor_val >= sensor_mapp[i+1]){
            a_index = i;
            b_index = i+1;

            x_a = sensor_mapp[a_index];
            x_b = sensor_mapp[b_index];
            y_a = dist_mapp[a_index];
            y_b = dist_mapp[b_index];

            distance = y_a +(sensor_val-x_a)*(y_b - y_a)/(x_b - x_a);
            break;
        }
        else      {
            distance = 0.07;
        }
    }
    return distance;
}





//-------------------------------------------------------------------------------------------------//
//----------------------- UPDATE  OBSTACLE TABLE BASED ON SENSOR MEASUREMENTS----------------------//
/*
  This function compute the distance of the robot to the detectable obstacles based on its infrared
  sensor measurements.
*/
//------------------------------------------------------------------------------------------------//

void update_obstacle_range_bearing(){

    for(int i=0;i<NB_SENSORS;i++) {

            distances[i] = wb_distance_sensor_get_value(ds[i]); //Read sensor values
            distances_after_transf[i] = map_sensor_to_dist(distances[i]);
            
            if (distances_after_transf[i] > min_range_detect){
                obstacles_table[0][i] = -1.0;
            }
            else {
                obstacles_table[0][i] = distances_after_transf[i];
            }

            obstacles_table[1][i] = sensor_orientations[i];
    }
}





//----------------------------------------------------------------------------------------------//
// ---------------------- INITIALIZE WEBOTS AND RECEIVER/EMITTER DEVICES -----------------------//
/*
  This function resets the robot device, gets its ID and initialize its sensor devices as well as
  emitter and receiver.
*/
//----------------------------------------------------------------------------------------------//


static void reset() 
{
	receiver2 = wb_robot_get_device("receiver");
	emitter2 = wb_robot_get_device("emitter");
	
	
	int i;
	char s[4]="ps0";
	for(i=0; i<NB_SENSORS;i++) {
		ds[i]=wb_robot_get_device(s);	     // the device name is specified in the world file
		s[2]++;				                     // increases the device number
	}
	robot_name=(char*) wb_robot_get_name(); 

	for(i=0;i<NB_SENSORS;i++)
              wb_distance_sensor_enable(ds[i],64);

	wb_receiver_enable(receiver2,64);

	//Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
	sscanf(robot_name,"epuck%d",&robot_id_u); // read robot id from the robot's name
	robot_id = robot_id_u%FLOCK_SIZE;	  // normalize between 0 and FLOCK_SIZE-1
	//printf("FLOCK 0 ROBOT id: %d\n", robot_id);
  
	for(i=0; i<FLOCK_SIZE; i++) 
	{
		initialized[i] = 0;		  // Set initialization to 0 (= not yet initialized)
	}

}





//----------------------------------------------------------------------------------------------------//
//----------------------FUNTION THAT SETS UPPER/LOWER LIMITS TO AN INPUT -----------------------------//
/*
  Function that keeps given int number within interval {-limit, limit}
*/
//----------------------------------------------------------------------------------------------------//

void limit(int *number, int limit) {
	if (*number > limit)
		*number = limit;
	if (*number < -limit)
		*number = -limit;
}




//---------------------------------------------------------------------------------------------------//
//----------------------- USE ODOMOETRY TO UPDATE LOCALIZATION --------------------------------------//
/*
  This function Updates robot position with wheel speeds based on the odometry model. 
*/
//---------------------------------------------------------------------------------------------------//

void update_self_motion(double msl, double msr) { 
  
	// Compute deltas of the robot
	float dr = msr * WHEEL_RADIUS * DELTA_T;
	float dl = msl * WHEEL_RADIUS * DELTA_T;

	float du = (dr + dl)/2.0;
	float dtheta = (dr - dl)/AXLE_LENGTH;
	
	
	// Compute new position in global frame

	my_position[0] += du*cosf(my_position[2] + dtheta*0.5);
	my_position[1] += du*sinf(my_position[2] + dtheta*0.5);
	my_position[2] += dtheta;

	// Keep orientation within 0, 2pi
	if (my_position[2] > 2*M_PI) my_position[2] -= 2.0*M_PI;
	if (my_position[2] < 0) my_position[2] += 2.0*M_PI;
}





//--------------------------------------------------------------------------------------------------//
//----------------------------- OBSTACLE AVOIDANCE ALGORITHM ---------------------------------------//
/*
  This function compute the wheel speeds based on the obstacle the robot detects around itself. The idea
  is to perform a wall following algorithm such that obstacle can be safely avoided without divering
  too much from the group.
*/
//--------------------------------------------------------------------------------------------------//

void OA(){

  double oa_x = 0.0, oa_y = 0.0;  // Obstacle avoidance weights
  double d_angle = 0.0;           // Orientation to avoid the obstacle
  double rate = 1000;             // Rate of the logistic regression
  double mid_point = 0.01;        // Mid.point of the logisitc regression
  double OA_weight = 1.0;       // Weight of the obstacle avoidance speeds 
  double max_OA = 1; 

  // Update the obstacle table to get information about location of obstacles
  update_obstacle_range_bearing(); 
 

  for (int i = 0; i<NB_SENSORS ; i++ ){

    // Read the table for each sensor
    double obst_range = obstacles_table[0][i]; 
    double obst_bearing = obstacles_table[1][i]; 


    if (obst_range != -1.0){ 

      //printf("Agent: %d, Sensor i: %d, obstacle detected\n",robot_id,  i);  

      //logistic function to give more or less importance to OA
      speed[robot_id][0] = speed[robot_id][0] * 1.0/(1 + exp(-rate*(obst_range-mid_point)) ) ; 
      speed[robot_id][1] = speed[robot_id][1] * 1.0/(1 + exp(-rate*(obst_range-mid_point)) ) ; 
      
      //double angle = atan2(speed[robot_id][1],speed[robot_id][0]);
      double angle = 0.0; 

      // determine the direcition the robot should follow for the wall following
      double abs_sens_angle = fabs(obst_bearing);
      double sign_sens_angle = (obst_bearing >= 0.0)-(obst_bearing < 0.0); 


      if (abs_sens_angle < 0.30 ){

              d_angle = angle- sign_sens_angle * (M_PI / 2.0 + 0.30) ; 
              oa_x += OA_weight*1./obst_range*cosf(d_angle);
              oa_y += OA_weight*1./obst_range*sinf(d_angle);

      }else if (abs_sens_angle <= 0.80){

        //d_angle = speed[robot_id][1]- sign_sens_angle * (M_PI / 2.0 + 0.80) ; 
        d_angle = angle- sign_sens_angle * (M_PI / 2.0 + 0.8) ; 
              oa_x += OA_weight*1./obst_range*cosf(d_angle);
              oa_y += OA_weight*1./obst_range*sinf(d_angle);

      }else if (abs_sens_angle <= 1.57){


        d_angle = angle -sign_sens_angle * (M_PI) ; 
              oa_x -= OA_weight*1./obst_range*cosf(d_angle);
              oa_y += OA_weight*1./obst_range*sinf(d_angle);


      }else {

        //d_angle = speed[robot_id][1]- sign_sens_angle * (M_PI / 4.0) ; 
        d_angle = angle - sign_sens_angle * (2.64) ; 
              oa_x -= OA_weight*1./obst_range*cosf(d_angle);
              oa_y += OA_weight*1./obst_range*sinf(d_angle);


      }
    }

  }
  
  if(fabs(oa_x) > 0.0 || fabs(oa_y) > 0.0){
  
     obst = 1; 
  }else{
         obst = 0; 
  }
  
  double oa_x_w = oa_x;
  double oa_y_w = oa_y;
  
  //printf("oa_x :  %f oa_y : %f\n", oa_x, oa_y);
  //saturate the OA
  if(fabs(oa_x_w) > max_OA || fabs(oa_y_w) > max_OA){
              if (fabs(oa_x_w) > fabs(oa_y_w)){
                 oa_x = max_OA * ((oa_x_w >= 0) - (oa_x_w < 0));
                 if (oa_x_w == 0){
         
                   oa_y = 0.0; 
         
                 }else{
                   oa_y = max_OA * fabs( oa_y_w / oa_x_w )* ((oa_y_w > 0) - (oa_y_w < 0));}
               }
               else{ 
                oa_y = max_OA * ((oa_y_w >= 0) - (oa_y_w < 0));
                if (oa_y_w == 0 ){
                  if(fabs(oa_x_w) > max_OA){
                    oa_x = max_OA; }
                }else{
                  oa_x = max_OA * fabs(oa_x_w / oa_y_w) * ((oa_x_w > 0) - (oa_x_w < 0));}
            }
          }
          
          //printf("robot %d, oa_x2 :  %f oa_y2 : %f\n",robot_id, oa_x, oa_y);
  // Add the computed speeds to the one based on FORMATION/FLOCKING (after logistic regression)      
  speed[robot_id][0] += oa_x;
  speed[robot_id][1] += oa_y;

}






//-----------------------------------------------------------------------------------------------------//
//------------------------------------ COMPUTES THE WHEEL SPEED ---------------------------------------//
/*
  This function compute the wheel speeds based on the translation and rotation speed of the robot using 
  its geoetrical properties
*/
//-----------------------------------------------------------------------------------------------------//

void compute_wheel_speeds()  
{
  
  float z = speed[robot_id][0]; // Heading direction 
  float x = speed[robot_id][1]; // Perpendicular direction
  
  float K1 = 0.25;   // Forward control coefficient
  float K2 = 3;      // Rotational control coefficient
  
  float range_to_target = sqrtf(x*x + z*z);   // Distance to the wanted position
  float bearing_to_target = atan2(x, z);

  
  // Compute forward control
  float u = K1*range_to_target*cosf(bearing_to_target);

  // Compute rotational control
  float w = K2*bearing_to_target + K1*sinf(bearing_to_target)*cosf(bearing_to_target);
  
  if (state == 2){
              u = 1.0 * ((u >= 0) - (u < 0)); 
              w = 0.0; 
  }
  
  // Convert to wheel speeds
  msl = (u - AXLE_LENGTH*w/2.0) * (1.0 / WHEEL_RADIUS);
  msr = (u + AXLE_LENGTH*w/2.0) * (1.0 / WHEEL_RADIUS);
  
  double msl_w = msl;
  double msr_w = msr;

  //printf(" agent:%d  msl before %f msr before : %f\n", robot_id, msl_w, msr_w);  

   
  // Make flock move at maximum speed while respecting the ratio between left and right wheel speeds
  if(fabs(msl_w) > MAX_SPEED_WEB || fabs(msr_w) >MAX_SPEED_WEB){
     if (fabs(msl_w) > fabs(msr_w)){
         msl = MAX_SPEED_WEB * ((msl_w >= 0) - (msl_w < 0));
         msr = MAX_SPEED_WEB * msr_w / msl_w * ((msr_w > 0) - (msr_w < 0));
     }
     else{ 
      msr = MAX_SPEED_WEB * ((msr_w >= 0) - (msr_w < 0));
      msl = MAX_SPEED_WEB * msl_w / msr_w * ((msl_w > 0) - (msl_w < 0));
    }
  }

}





//----------------------------------------------------------------------------------------------------------//
//--------------------------- REYNOLDS RULES OF FLOCKING ---------------------------------------------------//
/*
  This function Updates speed according to Reynold's rules of flocing based on range and bearing measurement 
  of all the robots of the same flock
*/
//----------------------------------------------------------------------------------------------------------//

void reynolds_rules() {
	
	// Center of all robots
	double center_neigh_x = 0.0;
 	double center_neigh_y = 0.0;

  // Seperation values
 	double separation_x = 0.0; 
 	double separation_y = 0.0; 


 	for (int i = 0; i < FLOCK_SIZE; i++){

    // Not considering itself
 		if (i != robot_id){
 		   
      // compute center of Flock 
 			center_neigh_x += relative_pos[i][0];
 			center_neigh_y += relative_pos[i][1];

      // Compute seperatioo of flock
 			double relative_dist = sqrt(pow(relative_pos[i][0],2)+pow(relative_pos[i][1],2)); 
			if( relative_dist < RULE2_THRESHOLD){
    			           if( relative_pos[i][0] != 0 && relative_pos[i][1]!=0){

				separation_x -= 1.0/relative_pos[i][0];
				separation_y -= 1.0/relative_pos[i][1];
				}
		          
		    }

 		}

 	}

 	//check if divide by N or (N-1) i.e. not consider self in the average 
 	center_neigh_x /= FLOCK_SIZE-1.0;
 	center_neigh_y /= FLOCK_SIZE-1.0;


 	double cohesion_x = center_neigh_x; 
 	double cohesion_y = center_neigh_y; 
 	
 //aggregation of all behaviors with relative influence determined by weights
 speed[robot_id][0] = cohesion_x * K_cohesion + separation_x * K_separation;
 speed[robot_id][1] = cohesion_y * K_cohesion + separation_y * K_separation;
     
 // Compute distance ot goal based on migratory urge           
 double dist_to_goal_x = (migr[0]-my_position[0]);
 double dist_to_goal_y = (migr[1]-my_position[1]);
	
 // Express distance to goal in robot frame
 double local_dist_to_goal_x = dist_to_goal_x*cosf(my_position[2]) + dist_to_goal_y*sinf(my_position[2]); 
 double local_dist_to_goal_y = -dist_to_goal_x*sinf(my_position[2]) + dist_to_goal_y*cosf(my_position[2]); 
 double range_to_goal = sqrt(local_dist_to_goal_x* local_dist_to_goal_x + local_dist_to_goal_y * local_dist_to_goal_y);

 // If robot close to goal, take a propational weight to the migrator urge
 if (range_to_goal < 0.1){

  	    speed[robot_id][0] += dist_to_goal_x * K_migration;
        speed[robot_id][1] += dist_to_goal_y * K_migration; 

	}
  // If robot far from goal, consider a normalized migratory urge
	else{

    	  speed[robot_id][0] += local_dist_to_goal_x / range_to_goal * K_migration;
    	  speed[robot_id][1] += local_dist_to_goal_y / range_to_goal * K_migration;
	}
}






//----------------------------------------------------------------------------------------------------------//
//------------------------------------- SEND PING ----------------------------------------------------------//
/*
 This function makes sure each robot sends a ping message, so the other robots can measure relative range 
 and bearing to the sender. the message contains the robot's name and to whom it supposed to go.
 The range and bearing will be measured directly out of message RSSI and direction.
*/
//----------------------------------------------------------------------------------------------------------//

void send_ping(void)  
{	
	char buffer_ping[255];
  char out[10];
	strcpy(out,robot_name);  // in the ping message we send the name of the robot.
	sprintf(buffer_ping,"#%d#%s",FLOCK_NUMBER,out);
	//printf("Sent: %s \n", buffer_ping);
	wb_emitter_send(emitter2,buffer_ping,strlen(buffer_ping)+1); 
}




//----------------------------------------------------------------------------------------------------------//
//------------------------------- PROCESS RECEIVED PING ---------------------------------------------------//
/*
 This function is processing all the received ping messages, and calculate range and bearing to the other robots
 the range and bearing are measured directly out of message RSSI and direction. Note that it only considers the
 ping comming with a header corresponding to their flock number to avoid getting information of other flocks of
 robot they are crossing
*/
//----------------------------------------------------------------------------------------------------------//

void process_received_ping_messages(void)
{
  const double *message_direction; // Direction of received signal
  double message_rssi;             // Received Signal Strength indicator
	double theta;                    // Orientation to the other robot
	double range;                    // Range to the other robot
	char *inbuffer;	                 // Buffer for the receiver node
  int other_robot_id;              // Robot id from which the message is coming 
  int channel;                     // Variable that will contain the header of the message

  // While some message are received
	while (wb_receiver_get_queue_length(receiver2) > 0) {
		
    // Read message
    inbuffer = (char*) wb_receiver_get_data(receiver2);

    // Obtain the header to save in the channel variable
		sscanf(inbuffer,"#%d",&channel);

    // If the message is comming from a robot of the same flock
		if (channel == FLOCK_NUMBER){

      // Process ping to obtain range and bearing
  		message_direction = wb_receiver_get_emitter_direction(receiver2);
  		message_rssi = wb_receiver_get_signal_strength(receiver2);
  		double y = message_direction[0];
  		double x = message_direction[2];

      // Orientaiton and range based on axis definition
      theta = -atan2(y,-x);
  		range = sqrt((1/message_rssi));
  		
  		other_robot_id = (int)(inbuffer[8]-'0');  // since the name of the sender is in the received message. Note: this does not work for robots having id bigger than 9!
      other_robot_id = other_robot_id%FLOCK_SIZE;

      // Store previous value
  		prev_relative_pos[other_robot_id][0] = relative_pos[other_robot_id][0];
  		prev_relative_pos[other_robot_id][1] = relative_pos[other_robot_id][1];

      // Make sure own relative pos is 0
  		relative_pos[robot_id][0] = 0.0; 
  		relative_pos[robot_id][1] = 0.0; 

      // Store new relative position of each other robot of the flock
  		relative_pos[other_robot_id][0] = range*cosf(theta);  // relative x pos
  		relative_pos[other_robot_id][1] = range*sinf(theta);   // relative y pos0.106762

  	  }

		wb_receiver_next_packet(receiver2);

	}
}



//---------------------------------------------------------------------------------------------------------------//
//----------------------------------- UPDATE LAPLACIAN COST MATRIX ----------------------------------------------//
/*
  This dunction update the cost matrix of the Laplacian controller such that the Hungarian algorithm can the assign
  optimally the position of each robot in the formation. 
*/
//---------------------------------------------------------------------------------------------------------------//

void update_cost_matrix(){

	// express the global formation from its center by subtracting it 
	double tempo_formation [2][FLOCK_SIZE]; 
	double c_form_x = 0.0; 
	double c_form_y = 0.0; 

	//convert the other robot's pose to the gloabl frame which is the frame of the formation
	double global_relative_pos[FLOCK_SIZE][2]; 
	double center_x = 0.0; 
	double center_y = 0.0; 


	for (int i = 0; i < FLOCK_SIZE; i ++){

	    global_relative_pos[i][0] = relative_pos[i][0]*cosf(my_position[2]) - relative_pos[i][1]*sinf(my_position[2]); 
	    global_relative_pos[i][1] = relative_pos[i][0]*sinf(my_position[2]) + relative_pos[i][1]*cosf(my_position[2]); 

	    //center of current robots 
	    center_x = center_x + global_relative_pos[i][0]/FLOCK_SIZE ;
	    center_y = center_y + global_relative_pos[i][1]/FLOCK_SIZE ;

	    //center of formation: 
	    c_form_x = c_form_x + initial_formation[0][i]/FLOCK_SIZE; 
		c_form_y = c_form_y + initial_formation[1][i]/FLOCK_SIZE; 

	}

	//get current robot's pose w.r.t. the center of the agents: 
	double c_pos[FLOCK_SIZE][2]; 

	for (int i = 0; i < FLOCK_SIZE; i ++){

		//shift origin of current robots to center of robots
		c_pos[i][0] = global_relative_pos[i][0] - center_x;  
	    c_pos[i][1] = global_relative_pos[i][1] - center_y;  

	    //shift origin of formation to center of formation
	    tempo_formation[0][i] = initial_formation[0][i] - c_form_x; 
		tempo_formation[1][i] = initial_formation[1][i] - c_form_y; 

	}


	//loop over the robots 
	for (int i = 0; i < FLOCK_SIZE; i ++){

		//loop over the formation possibilities 
		for (int j = 0; j <FLOCK_SIZE;j ++){

			cost_matrix[i][j] = (int) 1000*sqrt( (tempo_formation[0][j]-c_pos[i][0])*(tempo_formation[0][j]-c_pos[i][0]) + (tempo_formation[1][j]-c_pos[i][1])*(tempo_formation[1][j]-c_pos[i][1]) ) ;
		}
	}


}








//-------------------------------------------------------------------------------------------------------//
//----------------------------- GET ENCODER VALUE ------------------------------------------------------ //
/*
  This function reads the encoders values from the sensors
*/
//-------------------------------------------------------------------------------------------------------//
void controller_get_encoder()
{
  double time_step = wb_robot_get_basic_time_step() / 1000.0;
  // Store previous value of the left encoder
  _meas.prev_left_enc = _meas.left_enc;

  _meas.left_enc = wb_position_sensor_get_value(dev_left_encoder);

  double deltaleft = _meas.left_enc-_meas.prev_left_enc;
  // Store previous value of the right encoder
  _meas.prev_right_enc = _meas.right_enc;

  _meas.right_enc = wb_position_sensor_get_value(dev_right_encoder);

  double deltaright = _meas.right_enc-_meas.prev_right_enc;

  deltaleft  *= WHEEL_RADIUS;
  deltaright *= WHEEL_RADIUS;

  double omega = - ( deltaright - deltaleft ) / ( WHEEL_AXIS * time_step ); //ADDED MINUS TO TEST 
  double speed = ( deltaright + deltaleft ) / ( 2.0 * time_step );

  double a = _robot.pos.heading;

  double speed_wx = speed * cos(a);
  double speed_wy = speed * sin(a);
  
  _robot.speed.x = speed_wx;
  _robot.speed.y = speed_wy;
  _robot.pos.x += _robot.speed.x * time_step;
  _robot.pos.y += _robot.speed.y * time_step;
  _robot.pos.heading += omega * time_step;

  if(VERBOSE_ENC)
    printf("ROBOT %d enc : x: %g  y: %g heading: %g\n", 
    robot_id, _robot.pos.x, _robot.pos.y, _robot.pos.heading);
}






//-------------------------------------------------------------------------------------------------------//
//----------------------------- GET ACCELEROMETER VALUE ------------------------------------------------ //
/*
  This function reads the accelerometer values from the sensors
*/
//-------------------------------------------------------------------------------------------------------//

void controller_get_acc()
{
  double time_step = wb_robot_get_basic_time_step() / 1000.0;

  const double * acc_values = wb_accelerometer_get_values(dev_acc);
  memcpy(_meas.acc, acc_values, sizeof(_meas.acc));

  double accfront = ( _meas.acc[1] - _meas.acc_mean[1]);
  // double accside = ( _meas.acc[0] - _meas.acc_mean[0]);
  

  double heading_tmp = _robot.pos.heading;
  ///////HEADING/////////
  _meas.prev_left_enc = _meas.left_enc;
  _meas.left_enc = wb_position_sensor_get_value(dev_left_encoder);
  double deltaleft = _meas.left_enc-_meas.prev_left_enc;
  _meas.prev_right_enc = _meas.right_enc;
  _meas.right_enc = wb_position_sensor_get_value(dev_right_encoder);
  double deltaright = _meas.right_enc-_meas.prev_right_enc;
  deltaleft  *= WHEEL_RADIUS;
  deltaright *= WHEEL_RADIUS;
  double omega = -( deltaright - deltaleft ) / ( WHEEL_AXIS * time_step); //ADDED MINUS TO TEST
  _robot.pos.heading += omega * time_step; 
  ///////////////////////
  

  double delta_heading = _robot.pos.heading - heading_tmp; // - M_PI / 2;

  _robot.acc.x = accfront * cos(_robot.pos.heading);
  _robot.acc.y = accfront * sin(_robot.pos.heading);

  double spxtmp = _robot.speed.x;
  double spytmp = _robot.speed.y;
  _robot.speed.x = cos(delta_heading)*spxtmp - sin(delta_heading)*spytmp + _robot.acc.x * time_step;
  _robot.speed.y = sin(delta_heading)*spxtmp + cos(delta_heading)*spytmp + _robot.acc.y * time_step;
  _robot.pos.x += _robot.speed.x * time_step;
  _robot.pos.y += _robot.speed.y * time_step;

  if(VERBOSE_ACC)
    printf("ROBOT %d acc : x: %g  y: %g heading: %g\n", 
      robot_id, _robot.pos.x, _robot.pos.y, _robot.pos.heading);

}



//-------------------------------------------------------------------------------------------------------//
//----------------------------- COMPUTE MEAN ACCELERATION VALUE ---------------------------------------- //
/*
  This function computes the mean acceleration 
*/
//-------------------------------------------------------------------------------------------------------//

void controller_compute_mean_acc()
{
  static int count = 0;

  count++;
  const double * acc_values = wb_accelerometer_get_values(dev_acc);
  memcpy(_meas.acc, acc_values, sizeof(_meas.acc));

  if( count > 20 ) // Remove the effects of strong acceleration at the begining
  {
    for(int i = 0; i < 3; i++)
        _meas.acc_mean[i] = (_meas.acc_mean[i] * (count - 21) + _meas.acc[i]) / ((double) count-20);
  }
  int time_step = wb_robot_get_basic_time_step();
  if( count == (int) (TIME_INIT_ACC / (double) time_step) )
    printf("Accelerometer initialization Done ! \n");

  if(VERBOSE_ACC_MEAN)
        printf("ROBOT acc mean : %g %g %g\n", _meas.acc_mean[0], _meas.acc_mean[1] , _meas.acc_mean[2]);
}




//-------------------------------------------------------------------------------------------------------//
//--------------------------------- GET GPS VALUE ------------------------------------------------------ //
/*
  Tis function gets the gps measurements for the position and uses encoder for heading of the robot
*/
//-------------------------------------------------------------------------------------------------------//


void controller_get_gps(){

  memcpy(_meas.prev_gps, _meas.gps, sizeof(_meas.gps));
  const double * gps_position = wb_gps_get_values(dev_gps);
  memcpy(_meas.gps, gps_position, sizeof(_meas.gps));


}


