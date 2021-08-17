/*****************************************************************************/
/* File:         super_flocking.c                                           */
/* Version:      1.0                                                         */
/* Date:         6-June-21                                                   */
/* Description:  Supervisor of FLOCKING alorithm that compute performance	*/
/*               metrics based on round truth.                              */
/* Authors:      DIS group 12		                                         */
/*****************************************************************************/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>


//---------------------------------------------------------------------------------------------------//
//-------------------------	VARIABLES TO SET --------------------------------------------------------//

#define FLOCK_SIZE	5		                         // Number of robots in flock

int simulation_time = 90000;  // Duration of simulation for metric computation (OBSTACLES)
//int simulation_time = 30000; // Duration of simulation for metric computation (CROSSING)
bool accelerometer = false;    // Set to true if localization with accelerometer
//---------------------------------------------------------------------------------------------------//


#define VERBOSE_METRICS false					// Prints specific metrics (orientation, velocity and distance metrics)
#define VERBOSE_PERFORMANCE false   			// Prints  performance metric at time t


#define TIME_STEP	64		                         // [ms] Length of time step
#define MAX_SPEED  3.0                             // Max webots speed
#define TARGET_DIST  0.14                            //
#define MIN(a,b) (((a)<(b))?(a):(b))


/* Webots devices*/
WbNodeRef robs[FLOCK_SIZE];		                     // Robots nodes
WbFieldRef robs_trans[FLOCK_SIZE];	                 // Robots translation fields
WbFieldRef robs_rotation[FLOCK_SIZE];	             // Robots rotation fields
WbDeviceTag emitter;			                     // Single emitter

float loc[FLOCK_SIZE][3];		 // Location of each robot in the flock
float prev_loc[FLOCK_SIZE][3];   // Previous location of each robot in the flock
float speed[FLOCK_SIZE][2];		 // Speed X,Y of each robot in the flock
double speed_norm[FLOCK_SIZE];   // Speed of each robot in the flock
int offset;				         // Offset of robots number
int t;                           // time




/*
 * Initialize flock position and devices
 */
void reset(void) {
	wb_robot_init();

	char rob[7] = "epuck0";
	int i;
	for (i=0;i<FLOCK_SIZE;i++) {
		sprintf(rob,"epuck%d",i+offset);
		robs[i] = wb_supervisor_node_get_from_def(rob);
		robs_trans[i] = wb_supervisor_node_get_field(robs[i],"translation");
		robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
	}
}


/*
 * Get initial positions
 */
void init_poses(void)
{
        int i;
        for (i=0;i<FLOCK_SIZE;i++) {

		// Get data
		loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
		loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2]; // Z
		loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA

		// Run one step
		wb_robot_step(TIME_STEP);
	}
}


//*********************************************************** DIS METRICS FOR FLOCKING AND LAPLACIAN FORMATION *****************************************************************************

/*
 * Computes orientation metric for flocking
 */
double orientation_metric_flocking(){

	double N = FLOCK_SIZE;				// Flock size
	double N_pairs = N*(N-1.0)/2.0;     // Number of combination
	double acc_sum = 0.0;               // Variable that will accumulate the orientation error
	double H_diff = 0.0;				// Difference in heading


	//iterate over the number of pairs 
	for (int i = 0; i < N; i++){
    	for (int j = i + 1; j < N; j++) {
        	H_diff = loc[i][2] - loc[j][2];
        	acc_sum += fabs(H_diff)/M_PI;
    	}
	}

	return 1.0-1.0/N_pairs*acc_sum;
}




/*
 * Computes velocity metric for flocking
 */
double velocity_metric(){

	double N = FLOCK_SIZE;
	double max_dist = MAX_SPEED * 0.064 * 0.0205;

	//center of current position 
	double center_x = 0.0;
 	double center_y = 0.0;

 	for (int i = 0; i < N; i++){

 		center_x += loc[i][0]; 
 		center_y += loc[i][1]; 

 	}

 	center_x /= N;
 	center_y /= N;

 	//center of previous position

 	double prev_center_x = 0.0;
 	double prev_center_y = 0.0;

 	for (int i = 0; i < N; i++){

 		prev_center_x += prev_loc[i][0]; 
 		prev_center_y += prev_loc[i][1]; 

 	}

 	prev_center_x /= N;
 	prev_center_y /= N;
 	


 	//compute the sum 
	double accum = sqrt( (prev_center_x-center_x) * (prev_center_x-center_x) + (prev_center_y-center_y) * (prev_center_y-center_y));

	// Compute metric of travelled distance over max possible distance
 	double v = accum * 1.0/max_dist;
 	


	return v;
}






/*
 * Computes distance metric for flocking
 */
double dist_metric_flocking() {



	double N = FLOCK_SIZE;
	
	double acc_dist = 0.0;
	double accum = 0.0; 
	double dist1 = 0.0; 
	double acc_min = 0.0;
	double acc_min2 = 0.0; 
	double dist2 = 0.0; 
	double d = 0.0; 
	double N_pairs = N*(N-1.0)/2.0;


	//center of current position 
	double center_x = 0.0;
 	double center_y = 0.0;

 	for (int i = 0; i < N; i++){

 		center_x += loc[i][0]; 
 		center_y += loc[i][1]; 
 	}

 	center_x /= N;
 	center_y /= N;


 	// Compute the distance between all robots and the center of the flock
 	for(int i = 0; i < N; i++){

 		accum = 0.0; 
 		accum += (loc[i][0]-center_x) * (loc[i][0]-center_x);
    	accum += (loc[i][1]-center_y) * (loc[i][1]-center_y);
  		accum = sqrt(accum);

 		acc_dist += accum;	
 	}

 	// Normalize distance by the flock size
 	acc_dist /= N; 

 	// Compute the first part of the distance metric 
 	dist1 = 1.0/ (1.0 + acc_dist);


 	// Compute the distance between each robot
	for (int i = 0; i < N; i++){
          	for (int j = i + 1; j < N; j++) {

    	
    		
    		double accum2 = sqrt((loc[i][0]-loc[j][0]) * (loc[i][0]-loc[j][0]) + (loc[i][1]-loc[j][1]) * (loc[i][1]-loc[j][1])) ;
    		  			
                      double val_1 = accum2/TARGET_DIST; 
                      double val_2 =  1.0 / ((1-TARGET_DIST+accum2)*(1-TARGET_DIST+accum2)); 

        	acc_min += fmin( val_1 , val_2);
        	acc_min2 += MIN(val_1,val_2); 
        	
        	
        	
        	
      	}
	}

	// Normalize by the flock size
	dist2 = acc_min/N_pairs; 

	// Compute distance metric
	//printf("time:%d, cohesion  : %f\n", t, dist1);
        	//printf("time:%d, separation    : %f\n", t,  dist2);
 	d = dist1 * dist2; 

 	return d; 

}



/*
 * Computes performance for the flocking algorithm
 */
double comp_fitness(){

	double orientation = orientation_metric_flocking();
	double vel = velocity_metric();
	double dist = dist_metric_flocking();

	return orientation * vel * dist;

}


// *********************************************************************************************************************************************************************************************

/*
 * Main function.
 */
 
int main(int argc, char *args[]) {

	int i;			// Index
	reset();
    init_poses();
    int iter=0;
    double performance_metric = 0.0;
	double average_performance;
	int starting_time = 0;

	if (accelerometer){
		starting_time = 5000;
		simulation_time += 5000;
	}
	

	// Compute reference fitness values

		
	while (t <= simulation_time){
		wb_robot_step(TIME_STEP);
		
		if (t % 1 == 0) {
			for (i=0;i<FLOCK_SIZE;i++) {
				// Get data

				prev_loc[i][0] = loc[i][0]; // X
				prev_loc[i][1] = loc[i][1]; // Z
				prev_loc[i][2] = loc[i][2];// THETA

				loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
				loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2]; // Z
				loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA

				speed[i][0] = ( loc[i][0] - prev_loc[i][0] ) / TIME_STEP;
				speed[i][1] = ( loc[i][1] - prev_loc[i][1] ) / TIME_STEP;

				speed_norm[i] = sqrt( speed[i][0] * speed[i][0] + speed[i][1] * speed[i][1] );
			
    			}
	
			
		}




    	// Compute fitness of the time step
		double M_flocking = comp_fitness();

		if (VERBOSE_PERFORMANCE) {
			printf("time:%d, Flocking Performance: %f\n", t, M_flocking);
		}


		if (t>= starting_time){

			// compute the number of time step
	    	iter ++;
			// Compute average performance
			performance_metric += M_flocking;

			average_performance = performance_metric/iter;
			printf("Average performance %f\n", average_performance);
		}

		

		if (VERBOSE_METRICS){
			double orientation = orientation_metric_flocking();
			double vel = velocity_metric();
			double dist = dist_metric_flocking();
			printf("time:%d, orientation Performance: %f\n", t, orientation);
			printf("time:%d, vel Performance: %f\n", t, vel);
			printf("time:%d, Dist Performance: %f\n", t, dist);
			
			
		}
		
		t += TIME_STEP;
	}

}
