#include <gsl/gsl_blas.h>

#include "kalman_variable.h"

/*  Define kalman variables  */

double timestep;

gsl_vector* state;
gsl_matrix* cov;
gsl_matrix* A;
gsl_matrix* B;
gsl_matrix* R;
gsl_matrix* C;
gsl_matrix* Q;
