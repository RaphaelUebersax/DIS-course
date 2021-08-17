#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/receiver.h>
#include <webots/supervisor.h>

#define FLOCK_SIZE  1   // Number of robots in flock
#define TIME_STEP 16    // [ms] Length of time step

typedef struct 
{
  double x;
  double y;
  double heading;
} pose_t;

double compute_localization_error(
  double true_x, double true_y, 
  double estimated_x, double estimated_y);


WbNodeRef robs[FLOCK_SIZE];           // Robots nodes
WbFieldRef robs_trans[FLOCK_SIZE];    // Robots translation fields
WbFieldRef robs_rotation[FLOCK_SIZE]; // Robots rotation fields
// WbDeviceTag emitter;                  // Single emitter
WbDeviceTag receiver;                  // Single receiver

float loc[FLOCK_SIZE][3];     // True location of everybody in the flock

// Estimated locations of everybody in the flock
float estimated_loc_enc[FLOCK_SIZE][2]; // odometry with accelerometers
float estimated_loc_acc[FLOCK_SIZE][2]; // odometry with encoders
float estimated_loc_gps[FLOCK_SIZE][2]; // by gps
float estimated_loc_kalman[FLOCK_SIZE][2]; // by kalman

int t;

/*
 * Initialize flock position and devices
 */
void reset(void) {
  wb_robot_init();

  // emitter = wb_robot_get_device("emitter");
  receiver = wb_robot_get_device("receiver");

  char rob[7] = "ROBOT1"; // for one fixed robot
  int i;
  for (i=0;i<FLOCK_SIZE;i++) {
    robs[i] = wb_supervisor_node_get_from_def(rob);
    robs_trans[i] = wb_supervisor_node_get_field(robs[i],"translation");
    robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
  }
}

//********* DIS METRICS FOR LOCALIZATION ***********************************

double accuracy_metric_localization(float estimated_loc[FLOCK_SIZE][2]){

  double error = 0.0; // localization error
  
  for (int i = 0; i < FLOCK_SIZE; i++){
    // true position 
    error += compute_localization_error(
      loc[i][0], 
      loc[i][1], 
      estimated_loc[i][0], 
      estimated_loc[i][1]);
//      estimated_loc[i][0]+_pose_origin.x, 
//      estimated_loc[i][1]+_pose_origin.y);
  }

  return error;
}

double compute_localization_error(
  double true_x, double true_y, 
  double estimated_x, double estimated_y) {
  
  //compute the norm. 
  double norm = sqrt( powf(true_x-estimated_x, 2) + powf(true_y-estimated_y, 2) );
  return norm;
  
  }

// *************************************************************************

/*
 * Main function.
 */
 
int main(int argc, char *args[]) {

  int i;      // Index
//  float rob_x, rob_z, rob_theta;  // Robot position and orientation
  
  char *inbuffer; // buffer for the estimated pose

  // Localization error for different methods
  double loc_err_enc = 0; // encoders
  double loc_err_acc = 0; // accelerometers
  double loc_err_gps = 0; // gps
  double loc_err_kalman= 0; // kalman
  
  double loc_err_enc_sum = 0;
  double loc_err_acc_sum = 0;
  double loc_err_gps_sum = 0;
  double loc_err_kalman_sum = 0;
  
  // Number of estimations received, to average over.
  // Since the measurements are periodic, averaging over the measurements gives
  // the average error over time.
  int num_measurements = 0;
  
  reset();

  wb_receiver_enable(receiver, 1);
    
  for(;;) {
    wb_robot_step(TIME_STEP);
    
    //update the data every time step
    //if (t % 1 == 0) {
      for (i=0;i<FLOCK_SIZE;i++) {

        // Get the ground truth position.
        loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
        loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2]; // Z
        loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA
          
        // Get the positions estimated by the different localization metrics.
        while (wb_receiver_get_queue_length(receiver) > 0) { // if
          
          inbuffer = (char*) wb_receiver_get_data(receiver);
          
          sscanf(inbuffer,"###%f#%f_%f#%f_%f#%f_%f#%f", 
            &estimated_loc_enc[i][0], &estimated_loc_enc[i][1], // from encoders
            &estimated_loc_acc[i][0], &estimated_loc_acc[i][1], // from accelerometers
            &estimated_loc_gps[i][0], &estimated_loc_gps[i][1], // from gps
            &estimated_loc_kalman[i][0], &estimated_loc_kalman[i][1] // from kalman
          );
          
          //printf("EstimatedX %f, TrueX %f, EstimatedY %f, TrueY %f\n",
          // estimated_loc_acc[0][0],loc[0][0], estimated_loc_acc[0][1],loc[0][1]); 
          //  estimated_loc_kalman[0][0],loc[0][0], estimated_loc_kalman[0][1],loc[0][1]); 
          // estimated_loc_enc[0][0],loc[0][0], estimated_loc_enc[0][1],loc[0][1]); 
          wb_receiver_next_packet(receiver);
          
          // Compute the localization error for the estimated positions.
          num_measurements += 1;
          loc_err_enc = accuracy_metric_localization(estimated_loc_enc);
          loc_err_acc = accuracy_metric_localization(estimated_loc_acc);
          loc_err_gps = accuracy_metric_localization(estimated_loc_gps);
          loc_err_kalman = accuracy_metric_localization(estimated_loc_kalman);

          // Add the computed errors to a sum to compute the average error later on.
          loc_err_enc_sum += loc_err_enc;
          loc_err_acc_sum += loc_err_acc;
          loc_err_gps_sum += loc_err_gps;
          loc_err_kalman_sum += loc_err_kalman;
          
        }
      }
      
      if (t % 1000 == 0) // print every second
        printf("time:%d, Localization Accuracy Performance: enc: %f, acc: %f, gps: %f kalman: %f\n", 
          t, loc_err_enc, loc_err_acc, loc_err_gps, loc_err_kalman);
        
    //}


    // Print every second the average accuracy. 
    // Since the trajectories terminate via a "running" boolean, 
    //   the main controller will stop sending the estimations.
    if (t % 1000 == 0) {
      printf("time:%d, Average error: enc: %f, acc: %f, gps: %f, kalman: %f\n", t, 
        loc_err_enc_sum / num_measurements,
        loc_err_acc_sum / num_measurements,
        loc_err_gps_sum / num_measurements,
        loc_err_kalman_sum / num_measurements);}
        
        t += TIME_STEP;
  }
     
  

}
