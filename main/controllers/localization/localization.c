/*****************************************************************************/
/* File:         localization.c                                              */
/* Version:      3.0                                                         */
/* Date:         6-June-21                                                   */
/* Description:  Testing the Localization techniques on the trajectories     */
/*                                                                           */
/* Authors:      DIS group 12		                           */
/*****************************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>

#include <webots/emitter.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>

#include "utils.h"

// include trajectories file
#include "trajectories.h"

//--------------------------------------------------------------------------------------------------------------//

/* TRAJECTORY CHOICE TO COMPUTE THE LOCALIZATION PERFORMANCE */
#define TRAJECTORY_NO 1 // TODO: SET TO 1 OR 2 FOR TRAJECTORY_1 AND TRAJECTORY_2 RESPECTIVELY

//-------------------------------------------------------------------------------------------------------------//

/* FLOCK */
#define FLOCK_SIZE 1          // One robot only in the localization world.

// VERBOSE to print  intermediary results
#define VERBOSE_ENC false
#define VERBOSE_ACC false
#define VERBOSE_GPS false
#define VERBOSE_KALMAN false
#define VERBOSE_ACC_MEAN false
#define VERBOSE_KF false

/*CONSTANTS*/
#define TIME_INIT_ACC 5                    // Time in second

#define WHEEL_AXIS 	 0.057     // Distance between the two wheels in meter
#define WHEEL_RADIUS              0.0205    // Wheel radius (meters)

#define ABS(x) ((x>=0)?(x):-(x))

//-------------------------------------------------------------------------------------------------//
//----------------------------- Definition of structures ------------------------------------------//
//-------------------------------------------------------------------------------------------------//

typedef struct 
{
  double x;
  double y;
  double heading;
} pose_t;

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

WbDeviceTag emitter2;		// Handle for the emitter node

//-------------------------------------------------------------------------------------------------//
//----------------------------- GLOBAL VARIABLES --------------------------------------------------//
//-------------------------------------------------------------------------------------------------//

int cost_matrix[FLOCK_SIZE][FLOCK_SIZE];   // Cost matrix ufor Laplacian assignment
int local_urge = 1;                         // Local migration urge (true or false)
int robot_id = 1;	                  // Unique and normalized (between 0 and FLOCK_SIZE-1) robot ID
int robot_assignment;                       // Assignment of the robot in Laplacian controller
int assignment_vect[FLOCK_SIZE];            // Assugnement vector for all robots in Laplacian controller

// Covarinace matrix used for Kalman Filter
static double KF_cov[MMS][MMS] = {{0.001, 0, 0, 0},
                                  {0, 0.001, 0, 0},
                                  {0, 0, 0.001, 0},
                                  {0, 0, 0, 0.001}};

double last_gps_time_s = 0.0f;    // Time of last GPS measurement

// estimation of odometry by encoders
static pose_t _pose_odo_enc;
// estimation of odometry by accelerometers (+ heading from encoders)
static pose_t _pose_odo_acc, _speed_odo_acc, _acc_odo_acc;
// estimation of localization by gps only.
static pose_t _pose_gps;
// estimation of localization by kalman (of encoders and gps).
static pose_t _pose_kalman, _speed_kalman;


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

void initial_position();
void send_localization(); 
 
//-----------------------------------------------------------------------------------//
//--------------------------- MAIN FUNCTION -----------------------------------------//
/*
  The main computes the localization of the robotson the trajectories.
*/
//-----------------------------------------------------------------------------------//

int main(){ 

  wb_robot_init(); // Initialize webots variabes
  reset();         // Resetting the robot

  int time_step = wb_robot_get_basic_time_step(); // Time step of the simulator
  init_devices(time_step);                        // Initialize the different devices

  int iter = 0;           // initialize iteration to 0

  initial_position();     // initialize position
  bool running; // trajectory is running
 
  // Initialize the localization values; only x is different than zero. 
  _pose_odo_enc.x = _robot.pos.x;
  _pose_odo_acc.x = _robot.pos.x;
  _pose_gps.x = _robot.pos.x;
  _pose_kalman.x = _robot.pos.x;
 
  // Forever
  while (wb_robot_step(time_step) != -1) {
     //controller_get_gps();
     //printf("time: %f gps: x: %f, y: %f \n", wb_robot_get_time(), _pose_gps.x, _pose_gps.y);
     
    // Not on first iteration !
    if (iter!=0) {
    
      if(wb_robot_get_time() < TIME_INIT_ACC) {
        // Wait not to bias the accelerometers.
        wb_motor_set_velocity(dev_left_motor, 0);
        wb_motor_set_velocity(dev_right_motor, 0);
     
        controller_compute_mean_acc();
         
        //time_end_calibration = wb_robot_get_time(); // not used?
        //last_gps_time_s = wb_robot_get_time();
        continue; 
      }
      
      controller_get_encoder();
      
      controller_get_acc();

      // Update the covariance matrix for Kalman filter.
      KF_Update_Cov_Matrix((double) time_step/1000);
      double time_now_s = wb_robot_get_time();

      if (time_now_s - last_gps_time_s > 1.0f) {
        //printf("time now: %f,  last time: %f \n", time_now_s, last_gps_time_s);
        
        last_gps_time_s = floor(time_now_s);
        controller_get_gps();
  
        //printf("ACC1: %g %g %g\n", _robot.acc.x , _robot.acc.y, _robot.acc.heading);
        Kalman_Filter();
        //printf("ACC2: %g %g %g\n", _robot.acc.x , _robot.acc.y, _robot.acc.heading);
  
        if (VERBOSE_KALMAN)
          printf("ROBOT %d, pose after Kalman: %g %g %g\n\n",
            robot_id, _pose_kalman.x , _pose_kalman.y, _pose_kalman.heading);
            // robot_id, _robot.pos.x , _robot.pos.y, _robot.pos.heading);
      }
      
      send_localization();

    }

    // Set iter to 1 to execute localization from now on.
    iter = 1;
    if (TRAJECTORY_NO == 1)
      running = trajectory_1(dev_left_motor, dev_right_motor, TIME_INIT_ACC);
    else if (TRAJECTORY_NO == 2) 
      running = trajectory_2(dev_left_motor, dev_right_motor, TIME_INIT_ACC);
    else {
      printf("Undefined trajectory number %d: choose either 1 or 2", TRAJECTORY_NO);
      break;
    }

    
    if (!running){
      printf("Trajectory complete. \n");
      break;
      }

  }

  // End of the simulation.
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

  // Initialize GPS.
  dev_gps = wb_robot_get_device("gps");
  wb_gps_enable(dev_gps, 1000); //1000);

  // Initialize accelerometer.
  dev_acc = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(dev_acc, ts);

  // Initialize encoders.
  dev_left_encoder = wb_robot_get_device("left wheel sensor");
  dev_right_encoder = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(dev_left_encoder, ts);
  wb_position_sensor_enable(dev_right_encoder, ts);

  // Initialize motors.
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
  This function initializes the position of the robot based on the localization world.
*/
//------------------------------------------------------------------------------------------------------//

void initial_position() {
  _robot.pos.x = -2.9;
  _robot.pos.y = 0;
  _robot.pos.heading = 0;
}



//-------------------------------------------------------------------------------------------------------//
// ------------------------ UPDATE COVARIANCE MATRIX OF KALMAN FILTER -----------------------------------//
/*
  This function update the covariance matrix of the Kalman filter 
*/
//-------------------------------------------------------------------------------------------------------//

void KF_Update_Cov_Matrix(double ts) {
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
// ----------------------- SEND ESTIMATED POSITIONS TO SUPERVISOR -------------------------------------//
/*
  This function uses the a Kalman filter with the encoder and GPS measurement to infer the robot's 
  localization. 
*/
//-----------------------------------------------------------------------------------------------------//

void send_localization() {
  char buffer[255]; // Buffer for sending data
  // Send it out
  sprintf(buffer,"###%f#%f_%f#%f_%f#%f_%f#%f", 
    _pose_odo_enc.x, _pose_odo_enc.y, // from encoders
    _pose_odo_acc.x, (-1)*-_pose_odo_acc.y, // from accelerometers
    _pose_gps.x, _pose_gps.y,  // from gps
    _pose_kalman.x, _pose_kalman.y  // from kalman
  );
  
  //printf("Sent: %s \n", buffer);
  
  wb_emitter_send(emitter2, buffer, strlen(buffer));
}

//-----------------------------------------------------------------------------------------------------//
// ----------------------- COMPUTE POSITION USING KALMAN FILTER ---------------------------------------//
/*
  This function uses the a Kalman filter with the encoder and GPS measurement to infer the robot's 
  localization.
*/
//-----------------------------------------------------------------------------------------------------//

void Kalman_Filter() {
  static double X[MMS][MMS];

  X[0][0] = _pose_kalman.x;
  X[1][0] = _pose_kalman.y;
  X[2][0] = _speed_kalman.x;
  X[3][0] = _speed_kalman.y;
  
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
    
  _pose_kalman.x  = X_new[0][0];
  _pose_kalman.y  = X_new[1][0];
  _speed_kalman.x = X_new[2][0];
  _speed_kalman.y = X_new[3][0];
    
  if (VERBOSE_KF){
    printf("After\n");
    printf("Cov matrix\n");
    print_matrix(KF_cov, 4,4);
  
    printf("X matrix\n");
    print_matrix(X_new, 4,1);
  }
}


//----------------------------------------------------------------------------------------------//
// ---------------------- INITIALIZE WEBOTS AND RECEIVER/EMITTER DEVICES -----------------------//
/*
  This function resets the robot device.
*/
//----------------------------------------------------------------------------------------------//

static void reset() {
  emitter2 = wb_robot_get_device("emitter");
  robot_id = 1;
}


//-------------------------------------------------------------------------------------------------------//
//----------------------------- GET ENCODER VALUE ------------------------------------------------------ //
/*
  This function reads the encoders values from the sensors
*/
//-------------------------------------------------------------------------------------------------------//
void controller_get_encoder(){
  double time_step = wb_robot_get_basic_time_step() / 1000.0;
  // Store previous value of the left encoder
  _meas.prev_left_enc = _meas.left_enc;

  _meas.left_enc = wb_position_sensor_get_value(dev_left_encoder);

  double deltaleft = _meas.left_enc - _meas.prev_left_enc;
  // Store previous value of the right encoder
  _meas.prev_right_enc = _meas.right_enc;

  _meas.right_enc = wb_position_sensor_get_value(dev_right_encoder);

  double deltaright = _meas.right_enc - _meas.prev_right_enc;

  deltaleft  *= WHEEL_RADIUS;
  deltaright *= WHEEL_RADIUS;

  double omega = ( deltaright - deltaleft ) / ( WHEEL_AXIS * time_step ); //ADDED MINUS TO TEST 
  double speed = ( deltaright + deltaleft ) / ( 2.0 * time_step );

  double a = _pose_odo_enc.heading;

  double speed_wx = speed * cos(a);
  double speed_wy = speed * sin(a);
  
  _pose_odo_enc.x += speed_wx * time_step;
  _pose_odo_enc.y -= speed_wy * time_step;
  _pose_odo_enc.heading += omega * time_step;
  
  
   // Update kalman's estimates as well.
  _pose_kalman.x += speed_wx * time_step;
  _pose_kalman.y -= speed_wy * time_step;
  _pose_kalman.heading += omega * time_step;  

  if(VERBOSE_ENC)
    printf("ROBOT %d enc : x: %g  y: %g heading: %g ::::: gps %g %g \n", 
      robot_id, _pose_odo_enc.x, _pose_odo_enc.y, _pose_odo_enc.heading, _pose_gps.x, _pose_gps.y);
}


//-------------------------------------------------------------------------------------------------------//
//----------------------------- GET ACCELEROMETER VALUE ------------------------------------------------ //
/*
  This function reads the accelerometer values from the sensors
*/
//-------------------------------------------------------------------------------------------------------//

void controller_get_acc() {
  double time_step = wb_robot_get_basic_time_step() / 1000.0;

  const double * acc_values = wb_accelerometer_get_values(dev_acc);
  memcpy(_meas.acc, acc_values, sizeof(_meas.acc));

  double accfront = ( _meas.acc[1] - _meas.acc_mean[1]);
  //double accside = ( _meas.acc[0] - _meas.acc_mean[0]);
  

  double heading_tmp = _pose_odo_acc.heading;
  
  ///////HEADING/////////
  _pose_odo_acc.heading = _pose_odo_enc.heading;
  /*
  _meas.prev_left_enc = _meas.left_enc;
  _meas.left_enc = wb_position_sensor_get_value(dev_left_encoder);
  double deltaleft = _meas.left_enc-_meas.prev_left_enc;
  _meas.prev_right_enc = _meas.right_enc;
  _meas.right_enc = wb_position_sensor_get_value(dev_right_encoder);
  double deltaright = _meas.right_enc-_meas.prev_right_enc;
  deltaleft  *= WHEEL_RADIUS;
  deltaright *= WHEEL_RADIUS;
  double omega = -( deltaright - deltaleft ) / ( WHEEL_AXIS * time_step); //ADDED MINUS TO TEST
  _pose_odo_acc.heading += omega * time_step; */
  ///////////////////////
  
  double delta_heading = _pose_odo_acc.heading - heading_tmp; // - M_PI / 2;

  _acc_odo_acc.x = accfront * cos(_pose_odo_acc.heading);
  _acc_odo_acc.y = accfront * sin(_pose_odo_acc.heading);

  double spxtmp = _speed_odo_acc.x;
  double spytmp = _speed_odo_acc.y;
  _speed_odo_acc.x = cos(delta_heading)*spxtmp - sin(delta_heading)*spytmp + _acc_odo_acc.x * time_step;
  _speed_odo_acc.y = sin(delta_heading)*spxtmp + cos(delta_heading)*spytmp + _acc_odo_acc.y * time_step;
  _pose_odo_acc.x += _speed_odo_acc.x * time_step;
  _pose_odo_acc.y -= _speed_odo_acc.y * time_step;

  if(VERBOSE_ACC)
    printf("ROBOT %d acc : x: %g  y: %g heading: %g\n", 
      robot_id, _pose_odo_acc.x, _pose_odo_acc.y, _pose_odo_acc.heading);

}


//-------------------------------------------------------------------------------------------------------//
//----------------------------- COMPUTE MEAN ACCELERATION VALUE ---------------------------------------- //
/*
  This function computes the mean acceleration 
*/
//-------------------------------------------------------------------------------------------------------//

void controller_compute_mean_acc() {
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
  Tis function gets the gps measurements for the position and heading of the robot
*/
//-------------------------------------------------------------------------------------------------------//

void controller_get_gps() {

  memcpy(_meas.prev_gps, _meas.gps, sizeof(_meas.gps));
  const double * gps_position = wb_gps_get_values(dev_gps);
  memcpy(_meas.gps, gps_position, sizeof(_meas.gps));
  
  _pose_gps.x = _meas.gps[0];  
  _pose_gps.y = _meas.gps[2];
}

