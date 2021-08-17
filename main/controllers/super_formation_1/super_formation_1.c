/*****************************************************************************/
/* File:         super_formation.c                                           */
/* Version:      1.0                                                         */
/* Date:         6-June-21                                                   */
/* Description:  Supervisor of FORMATION alorithm that compute performance	*/
/*               metrics based on round truth. This file is used as supervisor */
/*				 for the 2nd group of robot in the crossing world             */
/* Authors:      DIS group 12		                                         */
/*****************************************************************************/



#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/supervisor.h>

#include "../hungarian.h"







//-----------------------------------------------------------------------------------------------------//
//----------------------------------VARIABLE TO SET----------------------------------------------------//

#define FLOCK_SIZE	5		        			// Number of robots in flock

// TODO: DECIDE THE LAPLACIAN FORMATION                         
double initial_formation[2][FLOCK_SIZE] = {{0,0,0,-0.2,0.2},{0.2,-0.2,0,0,0}};                                     // DESIRED FORMATION !!! (5 robots)


int offset = FLOCK_SIZE;      // Offset number to consider from roffset to offset+FLOCK_SIZE
int simulation_time = 30000;  // Duration of simulation for metric computation (CROSSING)
bool accelerometer = false;    // Set to true if localization with accelerometer
//-----------------------------------------------------------------------------------------------------//







#define VERBOSE_METRICS false					// Prints specific metrics (velocity and distance metrics)
#define VERBOSE_PERFORMANCE false   			// Prints  performance metric at time t


#define TIME_STEP	64		        			// [ms] Length of time step
#define MAX_SPEED  3.0             			// Max webots speed
#define PI 3.14159265358979323846   			// Define constant PI 


/* Webots devide tags*/
WbNodeRef robs[FLOCK_SIZE];						// Robots nodes
WbFieldRef robs_trans[FLOCK_SIZE];				// Robots translation fields
WbFieldRef robs_rotation[FLOCK_SIZE];			// Robots rotation fields
WbDeviceTag emitter;							// Single emitter


float loc[FLOCK_SIZE][3];		                // Location of each robot in the flock
float prev_loc[FLOCK_SIZE][3];                  // Previous localisation of each robot in the flock
float speed[FLOCK_SIZE][2];                     // Speed X,Y of each robot in the flock
double speed_norm[FLOCK_SIZE];                  // Speed of each robot in the flock
double target_distance[FLOCK_SIZE];             // Distance to target
int t;                                          // time 
int cost_matrix [FLOCK_SIZE][FLOCK_SIZE];       // Cost matrix to compute ideal hungarian assignment






/*
 * Initialize flock position and devices
 */
void reset(void) {

	wb_robot_init();
	char rob[7] = "epuck0";
	int i;
	for (i=0;i<FLOCK_SIZE;i++) {
		sprintf(rob,"epuck%d",i+offset);
		printf("rob: %s\n", rob);
		robs[i] = wb_supervisor_node_get_from_def(rob);
		robs_trans[i] = wb_supervisor_node_get_field(robs[i],"translation");
		robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
	}
}




/*
 * Converts array with reference to readable matrix
 */
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




/*
 * Computes the idea cost based on hungarian algorithm for Laplacian controller
 */
double compute_ideal_cost(){

	hungarian_problem_t p;

	int *r = (int*)cost_matrix;
	int** m = array_to_matrix(r,FLOCK_SIZE,FLOCK_SIZE);

		/* initialize the gungarian_problem using the cost matrix*/
	hungarian_init(&p, m , FLOCK_SIZE,FLOCK_SIZE, HUNGARIAN_MODE_MINIMIZE_COST) ;

	/* solve the assignement problem */
	hungarian_solve(&p);
	   
	int assignment_vect[FLOCK_SIZE]; 
	double cost_sum = 0;


	  //loop over the agents
	for (int i = 0; i < FLOCK_SIZE; i++){
	    //loop over the possible formation spots 
	  for (int j = 0; j < FLOCK_SIZE; j++){

	    if (*(p.assignment[i]+j) == 1 ){

	        //save the new assignment j of robot i 
	        assignment_vect[i] = j; 
	    }
	  }
	}

		  //loop over the agents
	for (int i = 0; i < FLOCK_SIZE; i++){


	  	cost_sum += cost_matrix[i][assignment_vect[i]]/1000.0;
	}

	/* free used memory */
	hungarian_free(&p);

	int idx;
	  for (idx=0; idx < FLOCK_SIZE; idx+=1) {
	    free(m[idx]);
	  }
	free(m);

	return cost_sum;

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



/*
 * Update the cost matrix using the hungarian algorithm to find distance to taget in the formation
 */
double update_cost_matrix(){


	// express the global formation from its center by subtracting it 
	double tempo_formation [2][FLOCK_SIZE]; 
	double rotated_formation [2][FLOCK_SIZE];
	double c_form_x = 0.0; 
	double c_form_y = 0.0; 

	//convert the other robot's pose to the gloabl frame which is the frame of the formation
	double center_x = 0.0; 
	double center_y = 0.0; 

	double interval = 100;
	double best_cost = 10000;


	for (int i = 0; i < FLOCK_SIZE; i++){

 		center_x += loc[i][0]; 
 		center_y += loc[i][1]; 

 		c_form_x = c_form_x + initial_formation[0][i]; 
		c_form_y = c_form_y + initial_formation[1][i]; 

 	}

 	center_x /= FLOCK_SIZE;
 	center_y /= FLOCK_SIZE;

 	c_form_x /= FLOCK_SIZE; 
	c_form_y /= FLOCK_SIZE;


	//get current robot's pose w.r.t. the center of the agents: 
	double c_pos[FLOCK_SIZE][2]; 

	for (int i = 0; i < FLOCK_SIZE; i ++){

		//shift origin of current robots to center of robots
		c_pos[i][0] = loc[i][0] - center_x;  
	    c_pos[i][1] = loc[i][1] - center_y;  

	    //shift origin of formation to center of formation
	    tempo_formation[0][i] = initial_formation[0][i] - c_form_x; 
		tempo_formation[1][i] = initial_formation[1][i] - c_form_y; 

	}


	for (double alpha=0; alpha < 2*PI; alpha += 2*PI/interval){

		for (int i=0; i<FLOCK_SIZE; i++){
			rotated_formation[0][i] = tempo_formation[0][i] * cosf(alpha) - tempo_formation[1][i] * sinf(alpha);
			rotated_formation[1][i] = tempo_formation[0][i] * sinf(alpha) + tempo_formation[1][i] * cosf(alpha);
		}

			//loop over the robots 
		for (int i = 0; i < FLOCK_SIZE; i ++){

			//loop over the formation possibilities 
			for (int j = 0; j <FLOCK_SIZE;j ++){

				cost_matrix[i][j] = (int) 1000*sqrt( (rotated_formation[0][j]-c_pos[i][0])*(rotated_formation[0][j]-c_pos[i][0]) + (rotated_formation[1][j]-c_pos[i][1])*(rotated_formation[1][j]-c_pos[i][1]) ) ;
			}
		}


		double cost = compute_ideal_cost();

		if (cost < best_cost) {
			best_cost = cost;
		}

	}

	return best_cost;
}






//*********************************************************** DIS METRICS AND LAPLACIAN FORMATION *****************************************************************************


/*
 * Compute the distance metric for formation controller
 */
double dist_metric_gt_formation() {

	double d = update_cost_matrix();	
 	return 1.0/(1.0 + 1.0/FLOCK_SIZE * d);
}




/*
 * Computes the velocity metric for formation controller
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

	
	double accum = sqrt( (prev_center_x-center_x) * (prev_center_x-center_x) + (prev_center_y-center_y) * (prev_center_y-center_y));
	// Compute metric of travelled distance over max possible distance
	
	// Divide travelled distance by the maximum possible travelled distance
 	double v = accum * 1.0/max_dist;

	return v;
}




/*
 * Computes the formation fitness
 */
double comp_fitness(){
	
	double vel = velocity_metric();
	double dist = dist_metric_gt_formation();
	return vel * dist;
}




// *********************************************************************************************************************************************************************************************

/*
 * Main function.
 */
 
int main(int argc, char *args[]) {

	int i;			// Index
	reset();
	double performance_metric = 0.0;
	double average_performance;
	int iter = 0;
	init_poses();
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


				// Update localization based on ground truth
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
		double M_formation = comp_fitness();

		if (VERBOSE_PERFORMANCE) {
			printf("time:%d, Formation Performance: %f\n", t, M_formation);
		}

		if (t>= starting_time){

    	// compute the number of time step
    	iter ++;
    	
		// Compute average performance
		performance_metric += M_formation;
		average_performance = performance_metric/iter;

		printf("Time: %d, Average performance Groupe 1: %f\n",t, average_performance);
		}


		if (VERBOSE_METRICS){

			double vel = velocity_metric();
			double dist_gt = dist_metric_gt_formation();
			printf("time:%d, vel Performance: %f\n", t, vel);
			printf("time:%d, Dist Performance GT: %f\n", t, dist_gt);
		}
		
		t += TIME_STEP;

	}



	return 0;

}
