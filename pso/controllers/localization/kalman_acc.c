#include "kalman_acc.h"

#define STATE_DIM 4
#define MEAS_DIM 2
#define CONTROL_DIM 2


void init_kalman_acc(const position_t* initial_pos)
{
    timestep = wb_robot_get_basic_time_step() / 1000.;

    // Process matrix
    double A_data[] = {1, 0, timestep, 0,
                       0, 1, 0, timestep,
                       0, 0, 1, 0,
                       0, 0, 0, 1};

    // Control matrix
    double B_data[] = {0, 0,
                       0, 0,
                       timestep, 0,
                       0, timestep};

    // Measurement model matrix
    double C_data[] = {1, 0, 0, 0,
                       0, 1, 0, 0};

    // Measurement covariance matrix
    double Q_data[] = {.01, 0,
                       0, .01};

    // Input covariance matrix
    double R_data[] = {0.05, 0, 0, 0,
                       0, 0.05, 0, 0,
                       0, 0, 0.05, 0,
                       0, 0, 0, 0.05};
    // double R_data[] = {0.05, 0, 0, 0,
    //                    0, 0.05, 0, 0,
    //                    0, 0, 0.01, 0,
    //                    0, 0, 0, 0.01};

    double state_init[] = {initial_pos->x, initial_pos->y, 0, 0};
    double cov_init[] = {0.001, 0, 0, 0,
                         0, 0.001, 0, 0,
                         0, 0, 0.001, 0,
                         0, 0, 0, 0.001};

    A = array2matrix(A_data, STATE_DIM, STATE_DIM);
    B = array2matrix(B_data, STATE_DIM, CONTROL_DIM);
    R = array2matrix(R_data, STATE_DIM, STATE_DIM);
    gsl_matrix_scale(R, timestep);
    C = array2matrix(C_data, MEAS_DIM, STATE_DIM);
    Q = array2matrix(Q_data, MEAS_DIM, MEAS_DIM);

    state = array2vector(state_init, STATE_DIM);
    cov = array2matrix(cov_init, STATE_DIM, STATE_DIM);
}


void update_pos_kalman_acc(position_t* pos, double acc[2], double gps[3], bool meas_true)
{
    double gps_data[MEAS_DIM] = {gps[0], -gps[2]};
    double input_data[CONTROL_DIM] = {acc[1], -acc[0]};

    kalman_prediction_step_acc(pos, input_data);
    if (meas_true) kalman_correction_step_acc(pos, gps_data);
    
    if (VERBOSE_KALMAN)
		printf("(EST) Kalman with acceleration : %g %g %g\n", pos->x, pos->y, (pos->heading));
		// printf("Kalman basic with acceleration : %g %g %g\n", pos->x, pos->y, RAD2DEG(pos->heading));
}


/*  1. Update state with motion model  */
void kalman_prediction_step_acc(position_t* pos, double input_data[CONTROL_DIM])
{
    gsl_vector* input = gsl_vector_alloc(CONTROL_DIM);
    *input = gsl_vector_view_array(input_data, CONTROL_DIM).vector;
    // printf("\ninput = \n");gsl_vector_fprintf(stdout, input, "%lf");


    /*  1.1   state = A * prev_state + B * input  */
    gsl_vector* temp_state = gsl_vector_alloc(STATE_DIM);
    gsl_blas_dgemv(CblasNoTrans, 1.0, A, state, 0.0, temp_state);
    gsl_vector_memcpy(state, temp_state);
    gsl_blas_dgemv(CblasNoTrans, 1.0, B, input, 1.0, state);


    /*  1.2   cov = A * prev_cov * transpose(A) + R * timestep  */
    gsl_matrix* temp_cov = gsl_matrix_alloc(STATE_DIM, STATE_DIM);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, A, cov, 0.0, temp_cov);
    gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, temp_cov, A, 0.0, cov);
    gsl_matrix_add(cov, R);


    pos->x = state->data[0];
    pos->y = state->data[1];
}


/*  2. Compute correction if we have a GPS update  */
void kalman_correction_step_acc(position_t* pos, double meas_val[MEAS_DIM])
{
    gsl_vector* meas = gsl_vector_alloc(MEAS_DIM);
    *meas = gsl_vector_view_array(meas_val, MEAS_DIM).vector;


    /*  2.1   K = cov * transpose(C) * inv(C * cov * transpose(C) + Q)  */
    gsl_matrix* covCT = gsl_matrix_alloc(STATE_DIM, MEAS_DIM);  // cov * transpose(C)
    gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, cov, C, 0.0, covCT);
    gsl_matrix* CcovCT = gsl_matrix_alloc(MEAS_DIM, MEAS_DIM);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, C, covCT, 0.0, CcovCT);
    gsl_matrix_add(CcovCT, Q);
    gsl_matrix* inv_CcovCT = invert_matrix(CcovCT, MEAS_DIM);
    gsl_matrix* K = gsl_matrix_alloc(STATE_DIM, MEAS_DIM);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, covCT, inv_CcovCT, 0.0, K);


    /*  2.2   state = state + K * ( meas_val - C * state)  */
    gsl_vector* C_state = gsl_vector_alloc(MEAS_DIM);
    gsl_blas_dgemv(CblasNoTrans, 1.0, C, state, 0.0, C_state);
    gsl_vector* meas_Cstate = gsl_vector_alloc(MEAS_DIM);
    gsl_vector_memcpy(meas_Cstate, meas);
    gsl_vector_sub(meas_Cstate, C_state);
    gsl_blas_dgemv(CblasNoTrans, 1.0, K, meas_Cstate, 1.0, state);


    /*  2.3   cov = (I - K * C) * cov  */
    gsl_matrix* I_KC = gsl_matrix_alloc(STATE_DIM, STATE_DIM);
    gsl_matrix_set_identity(I_KC);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, -1.0, K, C, 1.0, I_KC);
    gsl_matrix* temp_cov = gsl_matrix_alloc(STATE_DIM, STATE_DIM);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, I_KC, cov, 0.0, temp_cov);
    gsl_matrix_memcpy(cov, temp_cov);

    pos->x = state->data[0];
    pos->y = state->data[1];
}