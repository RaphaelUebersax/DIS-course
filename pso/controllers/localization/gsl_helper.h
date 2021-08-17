#include <gsl/gsl_linalg.h>
#include <gsl/gsl_blas.h>

#ifndef GSL_HELPER
#define GSL_HELPER

gsl_vector* array2vector(double* array, int size);
gsl_matrix* array2matrix(double* array, int size_1, int size_2);
gsl_matrix* invert_matrix(gsl_matrix *matrix, int size);

#endif