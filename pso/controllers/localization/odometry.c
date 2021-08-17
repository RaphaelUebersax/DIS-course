#include "odometry.h"


/*GLOBAL*/
static double T;

static double acc_mean_sum[2] = {0., 0.};
static double acc_cal_tsum = 0;


// INITIALISATION

void init_odometry(int time_step)
{
	T = time_step / 1000.0;
}


void calibrate_acc(double* acc, double* acc_mean) 
{    
    double time_now_s = wb_robot_get_time();
	printf("\ntimenow: %g\n", time_now_s);
    if (time_now_s > 0.1) {
        acc_cal_tsum += 1;
        acc_mean_sum[0] += acc[0]; 
        acc_mean_sum[1] += acc[1]; 
        acc_mean[0] = acc_mean_sum[0] / acc_cal_tsum;
        acc_mean[1] = acc_mean_sum[1] / acc_cal_tsum;
        printf("Acc mean: %g %g %g\n", acc_mean[0], acc_mean[1], acc_mean[2]);
    }
}


// READING THE SENSORS

void get_acc(WbDeviceTag device, double* acc)
{
    memcpy(acc, wb_accelerometer_get_values(device), 3*sizeof(*acc));

    if (VERBOSE_ACC)
        printf("ROBOT acc : %g %g %g\n", acc[0], acc[1] , acc[2]);
}


void get_encoder(WbDeviceTag device_left, WbDeviceTag device_right, double* prev_left_enc, double* prev_right_enc, double* left_enc, double* right_enc)
{
    *prev_left_enc = *left_enc; // Store previous value
    *left_enc = wb_position_sensor_get_value(device_left);
    
    *prev_right_enc = *right_enc; // Store previous value
    *right_enc = wb_position_sensor_get_value(device_right);

    if (VERBOSE_ENC)
        printf("ROBOT enc : %g %g\n", *left_enc, *right_enc);
}


void get_gps(WbDeviceTag device, double* last_gps_time_sec, double* prev_gps, double* gps, bool* gps_true)
{
    double time_now_s = wb_robot_get_time();
    *gps_true = false;

    if (time_now_s - *last_gps_time_sec > 1) {
        *last_gps_time_sec = time_now_s;
        memcpy(prev_gps, gps, 3*sizeof(*gps));
        memcpy(gps, wb_gps_get_values(device), 3*sizeof(*gps));
        *gps_true = true;
        
        if (VERBOSE_GPS)
            printf("ROBOT absolute gps : %g %g %g\n", gps[0], gps[1], gps[2]);
    }
}


// UPDATING POSITION ESTIMATION

void update_pos_acc(position_t* pos, position_t* speed, const double acc[3], double acc_mean[3], double Dleft_enc, double Dright_enc)
{
	double acc_wx = acc[1] - acc_mean[1];
	double acc_wy = -(acc[0] - acc_mean[0]);

	speed->x = speed->x + acc_wx * T;
	speed->y = speed->y + acc_wy * T;

	pos->x = pos->x + speed->x * T;
	pos->y = pos->y + speed->y * T;

	//  Compute heading with encoders
    update_heading_enc(&(pos->heading), Dleft_enc, Dright_enc);
	
	if (VERBOSE_ACC)
		printf("(EST) ODO with acceleration : x=%g y=%g th=%g\n", pos->x, pos->y, pos->heading);

}


void update_pos_enc(position_t* pos, double Dleft_enc, double Dright_enc)
{
	// Rad to meter : Convert the wheel encoders units into meters
    Dleft_enc  = Dleft_enc * WHEEL_RADIUS;
	Dright_enc = Dright_enc * WHEEL_RADIUS;

	// Compute speeds : Compute the forward and the rotational speed
	double omega = (Dright_enc - Dleft_enc) / WHEEL_AXIS / T;
	double speed = (Dright_enc + Dleft_enc) / 2 / T;
	
	// Compute the speed into the world frame (A) 
	double speed_wx = speed * cos(pos->heading + M_PI/2);
	double speed_wy = speed * sin(pos->heading + M_PI/2);

	// Integration : Euler method
	pos->x = pos->x + speed_wx * T;
	pos->y = pos->y + speed_wy * T;
	pos->heading = pos->heading + omega * T;

	if (VERBOSE_ENC)
		printf("(EST) ODO with encoders : x=%g y=%g th=%g\n", pos->x, pos->y, pos->heading);
}


void update_pos_gps(position_t *pos, double* last_gps_time_sec, double* prev_gps, double* gps)
{
    double time_now_s = wb_robot_get_time();
    double delta_x =   gps[0] - prev_gps[0];
    double delta_y = -(gps[2] - prev_gps[2]);

    pos->x =  gps[0] + (time_now_s - *last_gps_time_sec) * delta_x;
    pos->y = -gps[2] + (time_now_s - *last_gps_time_sec) * delta_y;
    pos->heading = atan2(delta_y, delta_x) - M_PI / 2;
    restrict_heading(&(pos->heading));

    if (VERBOSE_GPS)
        printf("(EST) GPS : x=%g y=%g th=%g\n", pos->x, pos->y, pos->heading);
}


// UPDATE HEADING

void update_heading_enc(double* heading, double Dleft_enc, double Dright_enc)
{
	*heading += (Dright_enc - Dleft_enc) * WHEEL_RADIUS / WHEEL_AXIS;
	restrict_heading(heading);
}


void restrict_heading(double* heading)
{
	if (*heading < -M_PI)
		*heading += 2*M_PI;
	else if (*heading > M_PI)
		*heading -= 2*M_PI;
}
