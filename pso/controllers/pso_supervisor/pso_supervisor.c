#include <stdio.h>
#include <math.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/supervisor.h>
#include <webots/robot.h>

#include "../const.h" // TODO: put definitions in const.h file
#include "../supervisor/metrics.h"
#include "../pso/pso.h"


/* PSO definitions */
#define NB 1          // Number of neighbors on each side
#define LWEIGHT 2.0   // Weight of attraction to personal best
#define NBWEIGHT 2.0  // Weight of attraction to neighborhood best
#define VMAX 40.0     // Maximum velocity particle can attain
#define MININIT -20.0 // Lower bound on initialization value
#define MAXINIT 20.0  // Upper bound on initialization value
#define ITS 20        // Number of iterations to run


/* Neighborhood types */
#define STANDARD -1
#define RAND_NB 0
#define NCLOSE_NB 1
#define FIXEDRAD_NB 2

#define NEIGHBORHOOD STANDARD

#define FINALRUNS 5
#define RADIUS 0.8


WbNodeRef robots[ROBOTS_N];
WbFieldRef rob_trans[ROBOTS_N];    // Robots translation fields
WbFieldRef rob_rotation[ROBOTS_N]; // Robots rotation fields

WbDeviceTag emitter;
WbDeviceTag receiver;
float loc_abs[ROBOTS_N][3];
double *loc[ROBOTS_N];
double *rot[ROBOTS_N];

static int time_step;


void init_super();
void get_absolute_position();
int update_metric(int task);
char valid_locs(int rob_id, int *j);
void random_pos(int);
double fitness(double[DATASIZE]);


/*
 * Initialize supervisor for getting robot absolute position
 */
void init_super()
{
    wb_robot_init();
    time_step = wb_robot_get_basic_time_step();


    receiver = wb_robot_get_device("receiver");
    wb_receiver_enable(receiver, time_step);

    emitter = wb_robot_get_device("emitter");   
    

    if (ROBOTS_N > 10)
        printf("==== ERROR ====\n More than 10 ROBOTS");

    char robot_name[7];
    for (int i = 0; i < ROBOTS_N; i++)
    {
        sprintf(robot_name, "epuck%d", i);
        // printf("Robot name %s \n", robot_name);
        robots[i] = wb_supervisor_node_get_from_def(robot_name);
        rob_trans[i] = wb_supervisor_node_get_field(robots[i], "translation");
        rob_rotation[i] = wb_supervisor_node_get_field(robots[i], "rotation");

        loc[i] = wb_supervisor_field_get_sf_vec3f(rob_trans[i]);
        rot[i] = wb_supervisor_field_get_sf_rotation(rob_rotation[i]);
    }
}


void get_absolute_position()
{
    for (int i = 0; i < ROBOTS_N; i++)
    {
        loc_abs[i][0] = wb_supervisor_field_get_sf_vec3f(rob_trans[i])[0];       // X
        loc_abs[i][1] = -wb_supervisor_field_get_sf_vec3f(rob_trans[i])[2];      // Z
        loc_abs[i][2] = wb_supervisor_field_get_sf_rotation(rob_rotation[i])[3]; // THETA
    }
}


int update_metric(int task)
{
    // Compute the metric for the corresponding task
    // TO DO: add localization metric to merge supervisors ?
    if (task == 1)
    {
        return update_flocking_metric(loc_abs);
    }
    else if (task == 2)
    {
        return update_formation_metric(loc_abs);
    }
    
    printf("YOU SHOULD NOT BE HERE");
    return 0;
}


/* MAIN - Distribute and test controllers */
int main()
{
    if (PSO) {
        double weights[DATASIZE];    // Optimized result
        double buffer[255]; // Buffer for emitter
        int i, j;        // Counter variables

        /* Initialisation */
        init_super();
        wb_robot_step(256);


        double fit;         // Fitness of the current FINALRUN
        double endfit;      // Best fitness over 10 runs
        double f;           // Evaluated fitness (modified by fitness() )
        double bestfit, bestw[DATASIZE];

        /* Evolve controllers */
        endfit = 0.0;
        bestfit = 0.0;

        // Do 10 runs and send the best controller found to the robot
        for (j = 0; j < 10; j++)
        {
            printf("Started PSO Optimization \n");

            // Get result of optimization
            pso(SWARMSIZE, NB, LWEIGHT, NBWEIGHT, VMAX, MININIT, MAXINIT, ITS, DATASIZE, &fitness, weights);

            printf("GOT PSO\n");

            fit = 0.0;
            // Run FINALRUN tests and calculate average
            for (i = 0; i < FINALRUNS; i++)
            {
                f = fitness(weights);
                fit += f;
            }
            fit /= FINALRUNS;


            // Check for new best fitness
            if (fit > bestfit)
            {
                bestfit = fit;
                copyParticle(bestw, weights);
            }

            printf("Performance of the best solution: %.3f\n", fit);
            endfit += fit / 10; // average over the 10 runs
            
            
            /* Position all robots around one random spot*/
            random_pos(ROBOTS_N);
            
        }

        printf("~~~~~~~~ Optimization finished.\n");
        printf("Best performance: %.3f\n", bestfit);
        printf("Average performance: %.3f\n", endfit);

        /* Send best controller to robots */

        for (j = 0; j < DATASIZE; j++)
            buffer[j] = bestw[j];
        buffer[DATASIZE] = 1000000;

        wb_emitter_send(emitter, (void *)buffer, (DATASIZE + 1) * sizeof(double));

        return 0;
    }
}


    // Makes sure no robots are overlapping        // TODO: add verification that there is no obstacle there
char valid_locs(int rob_id, int *j)
{
    for (int i = 0; i < ROBOTS_N; i++)
    {
        if (rob_id == i)
            continue;

        if (sqrt(pow(loc[i][0] - loc[rob_id][0], 2) + pow(loc[i][2] - loc[rob_id][2], 2)) <
            (2 * ROB_RADIUS + 0.01))
        {
            (*j)++;
            return 0;
        }
    }

    return 1;
}


// Randomly position specified robot
void random_pos(int numRobs)
{
    //printf("Setting random position for %d\n",rob_id);
    float leader_pos[2];
    leader_pos[0] = randIn(-ARENA_SIZE_X / 2.0, ARENA_SIZE_X / 2.0); // TODO: Check arena dimension (it is not square !)
    leader_pos[1] = randIn(-ARENA_SIZE_Y / 2.0, ARENA_SIZE_Y / 2.0);

    int i;
    for (i = 0; i < numRobs; i++)
    {
        // Note: changer la rotation perd le robot qui ne peut pas se retrouver
        // rot[i][0] = 0.0;
        // rot[i][1] = 1.0;
        // rot[i][2] = 0.0;
        // rot[i][3] = 2.0 * 3.14159 * randIn(0, 1);

        int j = 0;
        do
        {
            // disposition 2 by 2 0.5m apart
            loc[i][0] = leader_pos[0] + (i % 2) * 0.2 + j * 0.03; // add small values to find new places without robots and obstacles
            loc[i][2] = leader_pos[1] + (i / 2) * 0.2 + j * 0.03; // division will result in an integer
        } while (!valid_locs(i, &j));

        // TO DO: set migratory goal randomly
        
        //printf("%d at %.2f, %.2f\n", rob_id, loc[rob_id][0], loc[rob_id][2]);
        ///////////// random fly
        wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(robots[i], "translation"), loc[i]);
        // wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(robots[i], "rotation"), rot[i]);
    }
}


// Distribute fitness functions among robots
double fitness(double weights[DATASIZE])
{
    double buffer[255];

    ///* Position all robots around one random spot*/
    //random_pos(ROBOTS_N);

    buffer[0] = 1;
    buffer[1] = 0;

    for (int i = 0; i < DATASIZE; i++)
        buffer[i+2] = weights[i];

    // printf("Sending new weights: \n");
    // for (int i=0; i<DATASIZE; i++)
    //     printf("%f - ",weights[i]);
    // printf("\n");

    /* Send data to robots */
    // Message: always sending double[],
    // first value is: 0 for robot, 1 for supervisor
    // second value is: 0 for sending localisation position, 1 for sending ping, 0 for pso weights
    wb_emitter_send(emitter, (void *)buffer, (DATASIZE + 2) * sizeof(double)); // TODO: implement weights receiver at robot side

    reset_metric();
    bool running = true;

    while (running)
    {
        get_absolute_position();
        running = (bool)update_metric(TASK);
        wb_robot_step(time_step);
    }

    return get_metric();
}