/*
 * File:          supervisor_local.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/receiver.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>

#include "metrics.h"
#include "../const.h"
// #include "../localization/odometry.h"


WbNodeRef rob[ROBOTS_N];           // Robot node
WbFieldRef rob_trans[ROBOTS_N];    // Robots translation fields
WbFieldRef rob_rotation[ROBOTS_N]; // Robots rotation fields
WbDeviceTag receiver;              // Receiver for robot positions
WbDeviceTag emitter;               // Emitter for PSO supervisor


// Localization
float loc_abs[ROBOTS_N][3]; // Absolute Location of the robot
float loc_est[ROBOTS_N][3]; // Estimated position

static int time_step;


/*
 * Initialize supervisor for getting robot absolute position
 */
void init_super(void)
{
    wb_robot_init();
    time_step = wb_robot_get_basic_time_step();


    receiver = wb_robot_get_device("receiver");
    if (receiver == 0)
        printf("Missing receiver in supervisor\n");
    wb_receiver_enable(receiver, time_step);

    emitter = wb_robot_get_device("emitter");   
    

    char rob_name[7];
    for (int i = 0; i < ROBOTS_N; i++)
    {
        sprintf(rob_name, "epuck%d", i);
        // printf("Robot name %s \n", rob_name);
        rob[i] = wb_supervisor_node_get_from_def(rob_name);
        rob_trans[i] = wb_supervisor_node_get_field(rob[i], "translation");
        rob_rotation[i] = wb_supervisor_node_get_field(rob[i], "rotation");
    }


    // Find termination crit
    // Find if traj 1 or 2 and if goal reached in flocking or formation
}


void get_absolute_position(void)
{
    // Get data
    for (int i = 0; i < ROBOTS_N; i++)
    {
        loc_abs[i][0] = wb_supervisor_field_get_sf_vec3f(rob_trans[i])[0];       // X
        loc_abs[i][1] = -wb_supervisor_field_get_sf_vec3f(rob_trans[i])[2];      // Z
        loc_abs[i][2] = wb_supervisor_field_get_sf_rotation(rob_rotation[i])[3]; // THETA

        if (SUPERVISOR_VERBOSE_POSITION)
        {
            printf("(GT) Position of robot%d is x: %f, y: %f, theta: %f\n", i, loc_abs[i][0], loc_abs[i][1], loc_abs[i][2]);
            // printf("(GT) RPosition of robot%d is x: %f, y: %f, theta: %f\n", i, loc_abs[i][0]+2.9, loc_abs[i][1], loc_abs[i][2]+M_PI/2);
        }
    }

    //printf("(GT) Position is x: %f, y: %f, theta: %f\n", loc_abs[0][0], loc_abs[0][1], loc_abs[0][2]);
    //printf("(GT) vx: %g, vy: %g\n", wb_supervisor_node_get_velocity(rob[0])[0], -wb_supervisor_node_get_velocity(rob[0])[2]);
}


void get_info(void)
{
    int rob_nb, count = 0;
    float rob_x, rob_z, rob_theta;
    char *inbuffer;

    while (wb_receiver_get_queue_length(receiver) > 0 && count < ROBOTS_N)
    {
        inbuffer = (char *)wb_receiver_get_data(receiver);
        sscanf(inbuffer, "%d#%f#%f#%f", &rob_nb, &rob_x, &rob_z, &rob_theta);

        loc_est[rob_nb][0] = rob_x;
        loc_est[rob_nb][1] = rob_z;
        loc_est[rob_nb][2] = rob_theta;

        count++;
        wb_receiver_next_packet(receiver);
    }
}


void update_metric(int task)
{
    // Compute the metric for the corresponding task
    if (task == 0)
    {
        update_localization_metric(loc_abs, loc_est);
    }
    else if (task == 1)
    {
        update_flocking_metric(loc_abs);
    }
    else if (task == 2)
    {
        update_formation_metric(loc_abs);
    }
}


int main(void)
{
    init_super();

    while (1)
    {
        // Get absolute position of the robot
        get_absolute_position();

        // Receive estimated position of the robot
        if (TASK == 0)
            get_info();

        // Compute metric
        update_metric(TASK);

        wb_robot_step(time_step);
  }

}
