#include <gsl/gsl_linalg.h>
#include <gsl/gsl_blas.h>


gsl_vector *array2vector(double *array, int size)
{
    gsl_vector *vec = gsl_vector_alloc(size);
    gsl_vector *vec_view = gsl_vector_alloc(size);
    *vec_view = gsl_vector_view_array(array, size).vector;
    gsl_vector_memcpy(vec, vec_view);
    return vec;
}


gsl_matrix *array2matrix(double *array, int size_1, int size_2)
{
    gsl_matrix *mat = gsl_matrix_alloc(size_1, size_2);
    gsl_matrix *mat_view = gsl_matrix_alloc(size_1, size_2);
    *mat_view = gsl_matrix_view_array(array, size_1, size_2).matrix;
    gsl_matrix_memcpy(mat, mat_view);
    return mat;
}


gsl_matrix *invert_matrix(gsl_matrix *matrix, int size)
{
    int s;
    gsl_matrix *inv = gsl_matrix_alloc(size, size);
    gsl_permutation *p = gsl_permutation_alloc(size);

    gsl_linalg_LU_decomp(matrix, p, &s);
    gsl_linalg_LU_invert(matrix, p, inv);
    gsl_permutation_free(p);

    return inv;
}