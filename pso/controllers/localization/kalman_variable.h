#include <gsl/gsl_blas.h>

extern double timestep;

extern gsl_vector* state;
extern gsl_matrix* cov;
extern gsl_matrix* A;
extern gsl_matrix* B;
extern gsl_matrix* R;
extern gsl_matrix* C;
extern gsl_matrix* Q;