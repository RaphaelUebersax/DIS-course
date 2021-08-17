#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>


#include "../struct.h"
#include "../const.h"
#include "odometry.h"
#include "kalman_acc.h"
#include "kalman_vel.h"
#include "../pso/pso.h"


// Choose a localization technique
#define ENCODER 1
#define ACCELEROMETER 2
#define GPS 3
#define KALMAN_ACC 4
#define KALMAN_VEL 5

#define LOCALIZATION_METHOD KALMAN_VEL


// Calibrate Accelerometer
#define ACC_CAL false      // Enable accelerometer calibration (robot does not move for TIME_CAL seconds)
#define TIME_CAL 5         // Accelerometer calibration time (robot does not move for TIME_CAL seconds)


// For Obstacle Avoidance Behaviour
double braiten_weights[16] = {0.5, 0.4, 0.3, 0.1, -0.1, -0.3, -0.4, -0.5, -0.5, -0.4, -0.3, -0.1, 0.1, 0.3, 0.4, 0.5};
double last_obstacle_avoidance = -100.;
int flocking_group;

// For Flocking Behaviour
position_t previous_speed = {0, 0, 0};
static position_t flock_prev_rpos[ROBOTS_N], flock_rpos[ROBOTS_N];

// For Formation Behaviour
static const float rel_formation_pos[5][2] = {{0., 0.}, {-0.2, -0.2}, {-0.2, 0.2}, {0.2, -0.2}, {0.2, 0.2}}; // TO BE DEFINED
// static const float rel_formation_pos[5][2] = {{0., 0.}, {-0.4, 0}, {-0.2, 0}, {-0.2, -0.2}, {-0.2, 0.2}}; // TO BE DEFINED


static measurement_t meas;
static position_t initial_pos, pos, speed;

double last_gps_time_sec = 0.0f;
static int robot_id, robot_flock_id;
int time_step;

// For PSO weights
static double pso_weights[DATASIZE];
static bool new_weights = false;
static double dispersion_w;
static double cohesion_w;
static double migration_w;
static double dispersion_t;

WbDeviceTag dev_gps;
WbDeviceTag dev_acc;
WbDeviceTag dev_left_encoder;
WbDeviceTag dev_right_encoder;
WbDeviceTag dev_left_motor;
WbDeviceTag dev_right_motor;
WbDeviceTag emitter;
WbDeviceTag receiver;
WbDeviceTag dist_sensor[NB_SENSORS];


void init_variables()
{
    time_step = wb_robot_get_basic_time_step();

    // Initialize the robot id
    char *robot_name;
    robot_name = (char *)wb_robot_get_name();
    sscanf(robot_name, "epuck%d", &robot_id);
    robot_flock_id = robot_id % FLOCK_SIZE;
    flocking_group = robot_id / FLOCK_SIZE;

    // Initialize the position
    initial_pos = INIT_POS[robot_id];
    memcpy(&pos, &initial_pos, sizeof(initial_pos));

    // Initialize the gps measurements
    double gps_init[] = {pos.x, 0, -pos.y};
    memcpy(meas.gps, gps_init, sizeof(meas.gps));
    memcpy(meas.prev_gps, gps_init, sizeof(meas.gps));

    // Flocking parameters
    dispersion_w = DISPERSION_WEIGHT;
    cohesion_w = COHESION_WEIGHT;
    migration_w = MIGRATION_WEIGHT;
    dispersion_t = DISPERSION_THRESHOLD;
}


void init_devices()
{
    dev_gps = wb_robot_get_device("gps");
    wb_gps_enable(dev_gps, 1000);

    dev_acc = wb_robot_get_device("accelerometer");
    wb_accelerometer_enable(dev_acc, time_step);

    dev_left_encoder = wb_robot_get_device("left wheel sensor");
    dev_right_encoder = wb_robot_get_device("right wheel sensor");
    wb_position_sensor_enable(dev_left_encoder, time_step);
    wb_position_sensor_enable(dev_right_encoder, time_step);

    dev_left_motor = wb_robot_get_device("left wheel motor");
    dev_right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(dev_left_motor, INFINITY);
    wb_motor_set_position(dev_right_motor, INFINITY);
    wb_motor_set_velocity(dev_left_motor, 0.0);
    wb_motor_set_velocity(dev_right_motor, 0.0);

	receiver = wb_robot_get_device("receiver");
    emitter = wb_robot_get_device("emitter");
    wb_receiver_enable(receiver, time_step); // TODO: CHECK time_step

	char sensor_name[4] = "ps0";
	for (int i = 0; i < NB_SENSORS; i++)
	{
		dist_sensor[i] = wb_robot_get_device(sensor_name);
        wb_distance_sensor_enable(dist_sensor[i], time_step); // TODO: CHECK time_step
        sensor_name[2]++;
    }
}


void clamp(float *number, float limit)
{
	*number = (*number > limit) ? limit : (*number < -limit) ? -limit : *number;
}


double dist(double v1[2], double v2[2])
{
    return sqrtf(powf(v1[0] - v2[0], 2) + powf(v1[1] - v2[1], 2));
}


double dist_pos(position_t pos_1, position_t pos_2)
{
    double v_pos_1[2] = {pos_1.x, pos_1.y};
    double v_pos_2[2] = {pos_2.x, pos_2.y};
    return dist(v_pos_1, v_pos_2);
}


double norm_pos(position_t pos)
{
    return sqrtf(powf(pos.x, 2) + powf(pos.y, 2));
}


void vector_to_wheelspeed(float *msl, float *msr, float vx, float vy, bool use_range)
{
    // Compute wanted position in robots coordinate
    float heading = pos.heading + M_PI / 2;
    float x = vx * cosf(heading) + vy * sinf(heading);  // x in robot coordinates
    float y = vx * -sinf(heading) + vy * cosf(heading);  // y in robot coordinates

    float Ku = 0.3;                     // Forward control coefficient
    float Kw = 0.75;                     // Rotational control coefficient
    float range = sqrtf(x * x + y * y); // Distance to the wanted position
    float bearing = atan2(y, x);        // Orientation of the wanted position

    // Compute forward control
    float u = Ku * cosf(bearing);
    if (use_range)
        u *= range * 5;
        // u *= range * 3;

    // Compute rotational control
    // float w = Kw * bearing ;
    float w = Kw * bearing * 2;

    // Convert to wheel speeds!
    *msl = (u - AXLE_LENGTH * w / 2.0) * 2000;
    *msr = (u + AXLE_LENGTH * w / 2.0) * 2000;

    if (robot_id == 10) {
        printf("Formation MSL: %f MSR: %f\n", *msl, *msr);
        // printf("rob x: %f, rob y: %f, rob theta: %f (%f)\n", pos.x, pos.y, pos.heading + M_PI / 2, pos.heading);
        // printf("x: %f, y: %f\n", x, y);
        printf("range: %f, bearing: %f\n", range, bearing);
        // printf("u: %f, w: %f\n", u, w);
    }
    // limit(msl, MAX_SPEED);
    // limit(msr, MAX_SPEED);
    // printf("After Reynolds MSL: %d MSR: %d\n", *msl, *msr);
}


void braitenberg(float* msl, float* msr)
{
    double distances[NB_SENSORS]; // array keeping the distance sensor readings

    for (int sensor_nb = 0; sensor_nb < NB_SENSORS; sensor_nb++)
    {
        distances[sensor_nb] = wb_distance_sensor_get_value(dist_sensor[sensor_nb]);
        *msl += (distances[sensor_nb] * (-2) * braiten_weights[sensor_nb]);
        *msr += (distances[sensor_nb] * (-2) * braiten_weights[sensor_nb + NB_SENSORS]);
    }

    // printf("Braitenberg MSL: %f MSR: %f\n", *msl, *msr);
}


void reynolds(float* msl, float* msr)
{
    position_t migr = {MIGR[flocking_group].x, MIGR[flocking_group].y, 0};
	position_t flock_pos_avg = {0, 0, 0};	 // Flock average positions
	position_t flock_rpos_avg = {0, 0, 0};	 // Flock average positions
	position_t dispersion = {0, 0, 0}, cohesion = {0, 0, 0}, migration = {0, 0, 0};	 // Flock average positions
	position_t new_speed = {0, 0, 0};	 // Flock average positions
	// float flock_spd_avg[2] = {0, 0}; // Flock average speeds

    double norm;

	for (int i = 0; i < FLOCK_SIZE; i++)
	{
		if (i == robot_flock_id)
			continue;

        flock_rpos_avg.x += flock_rpos[i].x;
        flock_rpos_avg.y += flock_rpos[i].y;
        // flock_spd_avg.x += flock_spd[i][0];
        // flock_spd_avg.x += flock_spd[i][1];
		
        // Dispersion
        norm = norm_pos(flock_rpos[i]);
        if (norm < dispersion_t)
        {
            dispersion.x -= norm * sign(flock_rpos[i].x) / (abs(flock_rpos[i].x) + 0.5);
            dispersion.y -= norm * sign(flock_rpos[i].y) / (abs(flock_rpos[i].y) + 0.5);
        }
	}

	flock_rpos_avg.x /= (FLOCK_SIZE - 1);
	flock_rpos_avg.y /= (FLOCK_SIZE - 1);
	flock_pos_avg.x /= (FLOCK_SIZE - 1);
	flock_pos_avg.y /= (FLOCK_SIZE - 1);
	// flock_spd_avg.x /= (FLOCK_SIZE - 1);
	// flock_spd_avg.y /= (FLOCK_SIZE - 1);


    // Cohesion
    norm = norm_pos(flock_rpos_avg) + EPS;
    cohesion.x = flock_rpos_avg.x / norm;
    cohesion.y = flock_rpos_avg.y / norm;
    // cohesion.x = flock_rpos_avg.x * 5;
    // cohesion.y = flock_rpos_avg.y * 5;


    // Migration
    norm = dist_pos(migr, pos) + EPS;
    migration.x = (MIGR[flocking_group].x - pos.x) / norm;
    migration.y = (MIGR[flocking_group].y - pos.y) / norm;


    // Aggregate
    new_speed.x = dispersion.x * dispersion_w + cohesion.x * cohesion_w + migration.x * migration_w;
    new_speed.y = dispersion.y * dispersion_w + cohesion.y * cohesion_w + migration.y * migration_w;


    // Momentum (smoothen the trajectory)
    previous_speed.x = previous_speed.x * SPEED_MOMENTUM + new_speed.x * (1-SPEED_MOMENTUM);
    previous_speed.y = previous_speed.y * SPEED_MOMENTUM + new_speed.y * (1-SPEED_MOMENTUM);


    // Update msl and msr
    vector_to_wheelspeed(msl, msr, previous_speed.x, previous_speed.y, false);

    if (robot_id == 10) {
        printf("_\nCohesion %f, %f\n", cohesion.x * cohesion_w, cohesion.y * cohesion_w);
        printf("Dispersion %f, %f\n", dispersion.x * dispersion_w, dispersion.y * dispersion_w);
        printf("Migration %f, %f\n", migration.x * migration_w, migration.y * migration_w);
        printf("Reynold MSL: %f MSR: %f\n", *msl, *msr);
    }
}


void leader_follower_basic(float* msl, float* msr)
{
    // If leader
    if (robot_flock_id == LEADER_ID) {
        vector_to_wheelspeed(msl, msr, MIGR[flocking_group].x - pos.x, MIGR[flocking_group].y - pos.y, false);
        return;
    }
    
    // If follower
    vector_to_wheelspeed(msl, msr, flock_rpos[LEADER_ID].x, flock_rpos[LEADER_ID].y, true);
}


void leader_follower_positioned(float* msl, float* msr)
{
    // If leader
    if (robot_flock_id == LEADER_ID) {
        vector_to_wheelspeed(msl, msr, MIGR[flocking_group].x - pos.x, MIGR[flocking_group].y - pos.y, false);
        return;
    }
    
    // If follower
    vector_to_wheelspeed(msl, msr, flock_rpos[LEADER_ID].x + rel_formation_pos[robot_flock_id][0], flock_rpos[LEADER_ID].y + rel_formation_pos[robot_flock_id][1], true);
}


void formation(float* msl, float* msr)
{
    // leader_follower_basic(msl, msr);
    leader_follower_positioned(msl, msr);
}


void send_ping_message()
{
    //char robot_id_str[4];
    double buffer[255];
    // prepended 0 for robot identification, 1 for message ping
    //sprintf(robot_id_str, "0#1#%d", robot_id);  // change here
    buffer[0] = 0;
    buffer[1] = 1;
    buffer[2] = robot_id;

    /* Send ping */
    wb_emitter_send(emitter, (void *)buffer, 3 * sizeof(double));
    //wb_emitter_send(emitter, robot_id_str, strlen(robot_id_str));
}

void process_ping_messages(double* buffer)
{
    int sender_id, sender_flock_id, sender_flocking_group;

    const double *message_direction;
    double message_rssi;  // Received Signal Strength indicator
    double theta, range;
    double dx, dy, rx, ry;

    sender_id = buffer[2];
    sender_flock_id = sender_id % FLOCK_SIZE;
    sender_flocking_group = sender_id / FLOCK_SIZE;

    if (sender_flocking_group != flocking_group) {
        return;
    }

    message_direction = wb_receiver_get_emitter_direction(receiver);
    message_rssi = wb_receiver_get_signal_strength(receiver);

    dx = -message_direction[2];
    dy = -message_direction[0];

    theta = atan2(dy, dx) + pos.heading + M_PI / 2; // find the relative angle
    range = sqrt(1 / message_rssi);

    rx = range * cos(theta);        // relative x pos
    ry = range * sin(theta);        // relative y pos
    
    // printf("_\nReceive ping from robot %d, rx: %f, ry: %f\n", sender_id, rx, ry);
    // printf("theta: %f (%f), range: %f\n", theta, atan2(y, x), range);
    // printf("y %f, x %f\n", y, x);

    // Get position update
    flock_prev_rpos[sender_flock_id].x = flock_rpos[sender_flock_id].x;
    flock_prev_rpos[sender_flock_id].y = flock_rpos[sender_flock_id].y;
    flock_prev_rpos[sender_flock_id].heading = flock_rpos[sender_flock_id].heading;

    flock_rpos[sender_flock_id].x = rx;        // rel x pos
    flock_rpos[sender_flock_id].y = ry;        // rel y pos
    flock_rpos[sender_flock_id].heading = theta;        // rel theta pos

    // relative_speed[sender_id][0] = (1. / DELTA_T) * (flock_pos[sender_id].x - flock_prev_pos[sender_id].x);
    // relative_speed[sender_id][1] = (1. / DELTA_T) * (flock_pos[sender_id].y - flock_prev_pos[sender_id].y);
}


void receive_messages()
{
    // Buffer to receive the sender id
    double *inbuffer;
    int sender_type, message_type;

    while (wb_receiver_get_queue_length(receiver) > 0)
    {
        inbuffer = (double *)wb_receiver_get_data(receiver);
        sender_type = inbuffer[0];
        message_type = inbuffer[1];

        // If sender is a robot and message is ping message: process ping message
        if (sender_type == 0 && message_type == 1) {
            process_ping_messages(inbuffer);
        }
        // sender is supervisor and message type is weights: get new weights
        else if (sender_type == 1 && message_type == 0) {
            for (int i = 0; i < DATASIZE; i++) 
                pso_weights[i] = inbuffer[i+2] ;
            new_weights = true;
        }
        else {
            printf("Warning: Flockform controller should not receive this message\n");
        }
        wb_receiver_next_packet(receiver);
    }
}

void set_robot_weights() {

    printf("Setting new weights: \n");
    for (int i=0; i<DATASIZE; i++)
        printf("%f - ",pso_weights[i]);
    printf("\n");

    if (TASK == 1) {
        dispersion_w = pso_weights[0];
        cohesion_w = pso_weights[1];
        migration_w = pso_weights[2];
        dispersion_t = pso_weights[3];
    }
    else if (TASK == 2) {
        printf("Setting weights for formation not implemented yet");
    }
    else printf("Not setting weights for the localiation task");


}


int main()
{
    wb_robot_init();
    init_variables();
    init_devices();
    init_odometry(time_step);
    
    if (LOCALIZATION_METHOD == KALMAN_ACC)
        init_kalman_acc(&initial_pos);
    else if(LOCALIZATION_METHOD == KALMAN_VEL)
        init_kalman_vel(&initial_pos);

    while (wb_robot_step(time_step) != -1)
    {
        // printf("\n \n");
        // printf("\nNEW TIMESTEP\n");
        if (PSO && new_weights && TASK > 0) {
            set_robot_weights();
            new_weights = false;
        }

        // READ SENSORS
        get_encoder(dev_left_encoder, dev_right_encoder, &(meas.prev_left_enc), &(meas.prev_right_enc), &(meas.left_enc), &(meas.right_enc));
        get_acc(dev_acc, meas.acc);
        get_gps(dev_gps, &last_gps_time_sec, meas.prev_gps, meas.gps, &(meas.gps_true));

        // CALIBRATE ACCELEROMETER IF ENABLED
        double time_now_s = wb_robot_get_time();
        bool calibrating = ACC_CAL && (time_now_s < TIME_CAL);

        if (calibrating)
            calibrate_acc(meas.acc, meas.acc_mean);

        // UPDATE POSITION ESTIMATION
        if (LOCALIZATION_METHOD == ENCODER)
        {
            update_pos_enc(&pos, meas.left_enc - meas.prev_left_enc, meas.right_enc - meas.prev_right_enc);
        }
        else if (LOCALIZATION_METHOD == ACCELEROMETER && !calibrating)
        {
            update_pos_acc(&pos, &speed, meas.acc, meas.acc_mean, meas.left_enc - meas.prev_left_enc, meas.right_enc - meas.prev_right_enc);
        }
        else if (LOCALIZATION_METHOD == GPS)
        {
            update_pos_gps(&pos, &last_gps_time_sec, meas.prev_gps, meas.gps);
        }
        else if (LOCALIZATION_METHOD == KALMAN_ACC && !calibrating)
        {
            update_pos_kalman_acc(&pos, meas.acc, meas.gps, meas.gps_true);
            update_heading_enc(&(pos.heading), meas.left_enc - meas.prev_left_enc, meas.right_enc - meas.prev_right_enc);
        }
        else if (LOCALIZATION_METHOD == KALMAN_VEL)
        {
            double velocities[] = {wb_motor_get_velocity(dev_right_motor), wb_motor_get_velocity(dev_left_motor)};
            update_pos_kalman_vel(&pos, velocities, meas.gps, meas.gps_true);
        }

	    // printf("(EST) Kalman with velocity is x: %g, y: %g, theta: %g\n", pos.x, pos.y, pos.heading);
        

        // SEND & RECEIVE PINGS
        if (TASK == 1 || robot_flock_id == LEADER_ID)
            send_ping_message();
        if (TASK > 0)
            receive_messages();


        // DEFINE WHEEL SPEED
        float msl, msr; // Motor speed left and right
        float msl_w, msr_w;
        float braiten_msl = 0, braiten_msr = 0;
        float group_motion_msl = 0, group_motion_msr = 0;

        braitenberg(&braiten_msl, &braiten_msr);

        if (time_now_s - last_obstacle_avoidance < TIME_IN_OBSTACLE_AVOIDANCE)
        {
            msl = braiten_msl + BIAS_SPEED;
            msr = braiten_msr + BIAS_SPEED;
        }
        else if (abs(braiten_msl) + abs(braiten_msr) > 350)
        {
            printf("Start obstacle avoidance\n");
            last_obstacle_avoidance = time_now_s;
            msl = braiten_msl + BIAS_SPEED;
            msr = braiten_msr + BIAS_SPEED;
        }
        else if (TASK == 1)
        {
            reynolds(&group_motion_msl, &group_motion_msr);
            msl = group_motion_msl + braiten_msl + BIAS_SPEED / 4;
            msr = group_motion_msr + braiten_msr + BIAS_SPEED / 4;
        }
        else if (TASK == 2)
        {
            formation(&group_motion_msl, &group_motion_msr);
            msl = group_motion_msl;
            msr = group_motion_msr;
        }

        if (robot_id == 10)
            printf("Final MSL: %f MSR: %f\n", msl, msr);

        clamp(&msl, MAX_SPEED);
        clamp(&msr, MAX_SPEED);

        msl_w = msl * (MAX_SPEED_WEB - EPS) / MAX_SPEED;
        msr_w = msr * (MAX_SPEED_WEB - EPS) / MAX_SPEED;

        wb_motor_set_velocity(dev_left_motor, msl_w);
        wb_motor_set_velocity(dev_right_motor, msr_w);
    }
}
