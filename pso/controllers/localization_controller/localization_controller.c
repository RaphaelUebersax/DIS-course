#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>
#include <webots/emitter.h>


#include "../const.h"
#include "../struct.h"
#include "trajectories.h"
#include "odometry.h"
#include "kalman_acc.h"
#include "kalman_vel.h"


// Choose a localisation technique
#define ENCODER 1
#define ACCELEROMETER 2
#define GPS 3
#define KALMAN_ACC 4
#define KALMAN_VEL 5

#define LOCALIZATION_METHOD GPS


// Calibrate Accelerometer
#define ACC_CAL false      // Enable accelerometer calibration (robot does not move for TIME_CAL seconds)
#define TIME_CAL 5         // Accelerometer calibration time (robot does not move for TIME_CAL seconds)


static measurement_t meas;
static position_t pos, speed;
const position_t initial_pos = { -2.9, 0., -M_PI/2 }; // POSITION IN OUR REFERENTIAL (Y IS REVERSED Z)
double last_gps_time_sec = 0.0f;
static int robot_id;
int time_step;

WbDeviceTag dev_gps;
WbDeviceTag dev_acc;
WbDeviceTag dev_left_encoder;
WbDeviceTag dev_right_encoder;
WbDeviceTag dev_left_motor;
WbDeviceTag dev_right_motor;
WbDeviceTag emitter;		


void init_variables()
{
    time_step = wb_robot_get_basic_time_step();

    // Initialize the position
    memcpy(&pos, &initial_pos, sizeof(initial_pos));

    // Initialize the gps measurements
    double gps_init[] = {initial_pos.x, 0, -initial_pos.y};
    memcpy(meas.gps, gps_init, sizeof(meas.gps));
    memcpy(meas.prev_gps, gps_init, sizeof(meas.gps));
}

void init_devices()
{
    char *robot_name;
    robot_name = (char *)wb_robot_get_name();
    sscanf(robot_name, "epuck%d", &robot_id);

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

    emitter = wb_robot_get_device("emitter");
}

void send_position(position_t pos)
{
    char buffer[255]; // Buffer for sending data

    // Sending positions to the robots, comment the following two lines if you don't want the supervisor sending it
    // First 2 digits are identifiers, 0 for robot, and 0 for sending localisation position
    sprintf(buffer, "%d#%g#%g#%g", robot_id, pos.x, pos.y, pos.heading);
    wb_emitter_send(emitter, buffer, strlen(buffer));
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

        // READ SENSORS
        get_encoder(dev_left_encoder, dev_right_encoder, &(meas.prev_left_enc), &(meas.prev_right_enc), &(meas.left_enc), &(meas.right_enc));
        get_acc(dev_acc, meas.acc);
        get_gps(dev_gps, &last_gps_time_sec, meas.prev_gps, meas.gps, &(meas.gps_true));

        // CALIBRATE ACCELEROMETER IF ENABLED
        double time_now_s = wb_robot_get_time();
        bool calibrating = ACC_CAL && (time_now_s < TIME_CAL);
        if (calibrating) {
            calibrate_acc(meas.acc, meas.acc_mean);
        }

        // UPDATE POSITION ESTIMATION
        if (LOCALIZATION_METHOD == ENCODER)
            update_pos_enc(&pos, meas.left_enc - meas.prev_left_enc, meas.right_enc - meas.prev_right_enc);
        else if (LOCALIZATION_METHOD == ACCELEROMETER && !calibrating) {
            update_pos_acc(&pos, &speed, meas.acc, meas.acc_mean, meas.left_enc - meas.prev_left_enc, meas.right_enc - meas.prev_right_enc);
        } else if (LOCALIZATION_METHOD == GPS) {
            update_pos_gps(&pos, &last_gps_time_sec, meas.prev_gps, meas.gps);
        } else if (LOCALIZATION_METHOD == KALMAN_ACC && !calibrating) {
            update_pos_kalman_acc(&pos, meas.acc, meas.gps, meas.gps_true);
            update_heading_enc(&(pos.heading), meas.left_enc - meas.prev_left_enc, meas.right_enc - meas.prev_right_enc);
        } else if (LOCALIZATION_METHOD == KALMAN_VEL) {
            double velocities[] = {wb_motor_get_velocity(dev_right_motor), wb_motor_get_velocity(dev_left_motor)};
            update_pos_kalman_vel(&pos, velocities, meas.gps, meas.gps_true);
        }

        // SEND THE ESTIMATED POSITION TO THE SUPERVISOR FOR METRIC COMPUTATION
        send_position(pos);

        // SET WHEEL MOTOR SPEED TO FOLLOW TRAJECTORIES
        if (TRAJECTORY == 1)
            trajectory_1(dev_left_motor, dev_right_motor, ACC_CAL);
        else if (TRAJECTORY == 2)
            trajectory_2(dev_left_motor, dev_right_motor, ACC_CAL);
    }
}
