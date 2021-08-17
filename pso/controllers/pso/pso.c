#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/supervisor.h>

#include "pso.h"


#define VERBOSE true

/* Types of fitness evaluations */
#define EVOLVE 0     // Find new fitness
#define EVOLVE_AVG 1 // Average new fitness into total
#define SELECT 2     // Find more accurate fitness for best selection

/* Size of swarm data must be global variables */
int swarmsize;
int datasize;
int nb;
double (*fitness_fn)(double[]);

/* Particle swarm optimization function                                      */
/*                                                                           */
/* Parameters:                                                               */
/* n_swarmsize: number of particles in swarm                                 */
/* nb:          number of neighbors on each side of particle in neighborhood */
/* lweight:     max random value for local weight                            */
/* nbweight:    max random value for neighborhood weight                     */
/* vmax:        maximum velocity value                                       */
/* min:         minimum initial value of particle element                    */
/* max:         maximum initial value of particle element                    */
/* iterations:  number of iterations to run in the optimization              */
/* n_datasize:  number of elements in particle                               */
void pso(int n_swarmsize, int n_nb, double lweight, double nbweight, double vmax, double min, double max, int iterations, int n_datasize, double (*fitness_ptr)(double[]), double best[])
{
    double swarm[n_swarmsize][n_datasize];   // Swarm of particles
    double perf[n_swarmsize];                // Current swarm performance
    double localBest[n_swarmsize][n_datasize];   // Current local best data
    double localBestPerf[n_swarmsize];           // Current local best performance
    double localBestAge[n_swarmsize];            // Life length of best local swarm
    double nbBest[n_swarmsize][n_datasize];  // Current neighborhood best data 
    double nbBestPerf[n_swarmsize];          // Current neighborhood best performance
    double v[n_swarmsize][n_datasize];       // Speed
    int neighbors[n_swarmsize][n_swarmsize]; // Neighbor matrix
    int i, j, k;                             // FOR-loop counters
    double bestPerf;                         // Performance of evolved solution

    // Set global variables
    swarmsize = n_swarmsize;
    datasize = n_datasize;
    nb = n_nb;
    fitness_fn = fitness_ptr;

    srand(time(NULL));    // Seed the random generator

    // Setup neighborhood
    for (i = 0; i < swarmsize; i++)
    {
        for (j = 0; j < swarmsize; j++)
        {
            if (mod(i - j + nb, swarmsize) <= 2 * nb)
                neighbors[i][j] = 1;
            else
                neighbors[i][j] = 0;
            printf("%d ", neighbors[i][j]);
        }
        printf(" (%d)\n", i);
    }

    // Initialize the swarm
    for (i = 0; i < swarmsize; i++)
    {
        for (j = 0; j < datasize; j++)
        {
            swarm[i][j] = randIn(min, max);  // Random initial value in [min,max]
            localBest[i][j] = swarm[i][j];  // Best configurations are initially current configurations
            nbBest[i][j] = swarm[i][j];
            v[i][j] = randIn(-vmax, vmax);  // Random initial velocity
        }
    }

    // Best performances are initially current performances
    findPerformance(swarm, perf, NULL, EVOLVE, neighbors);
    for (i = 0; i < swarmsize; i++)
    {
        localBestPerf[i] = perf[i];
        localBestAge[i] = 1.0; // One performance so far
        nbBestPerf[i] = perf[i];
    }
    updateNBPerf(localBest, localBestPerf, nbBest, nbBestPerf, neighbors); // Find best neighborhood performances



    // Run optimization
    
    for (k = 0; k < iterations; k++)
    {

        if (VERBOSE) {
            printf("Iteration %d\n", k);
        }

        // Update preferences and generate new particles
        for (i = 0; i < swarmsize; i++)
        {
            for (j = 0; j < datasize; j++)
            {
                // Update velocities
                v[i][j] *= 0.6;
                v[i][j] += lweight * randIn(0,1) * (localBest[i][j] - swarm[i][j]) + nbweight * randIn(0,1) * (nbBest[i][j] - swarm[i][j]);

                // Move particles
                swarm[i][j] += v[i][j];
            }
        }

        // Find new performance
        findPerformance(swarm, perf, NULL, EVOLVE, neighbors);

        // Update best local performance
        updateLocalPerf(swarm, perf, localBest, localBestPerf, localBestAge);

        // Update best neighborhood performance
        updateNBPerf(localBest, localBestPerf, nbBest, nbBestPerf, neighbors);

        if (VERBOSE) {
            printf("Best performance of the iteration: %f\n", localBestPerf[argmax(localBestPerf, swarmsize)]);
        }
    }

    // Find best result achieved
    findPerformance(localBest, localBestPerf, NULL, SELECT, neighbors);
    bestPerf = bestInSwarm(localBest, localBestPerf, best);

    if (VERBOSE) {
        printf("### Best performance found ###\n");
        printf("Performance over %d iterations: %f\n", iterations, bestPerf);
    }
}


// Find the current performance of the swarm.
// Higher performance is better
void findPerformance(double swarm[swarmsize][datasize], double perf[swarmsize],
                     double age[swarmsize], char type,
                     int neighbors[swarmsize][swarmsize])
{
    double fit;
    int i, j;

    for (i = 0; i < swarmsize; i++)
    {
        if (type == EVOLVE_AVG)
        {
            fit = (*fitness_fn)(swarm[i]);
            perf[i] = ((age[i] - 1.0) * perf[i] + fit) / age[i];
            age[i]++;
        }
        else if (type == EVOLVE)
        {
            fit = (*fitness_fn)(swarm[i]);
            perf[i] = fit;
        }
        else if (type == SELECT)
        {
            perf[i] = 0.0;
            for (j = 0; j < 5; j++)
            {
                fit = (*fitness_fn)(swarm[i]);
                perf[i] += fit;
            }
            perf[i] /= 5.0;
        }
    }
}


// Update the best performance of a single particle
void updateLocalPerf(double swarm[swarmsize][datasize], double perf[swarmsize], double localBest[swarmsize][datasize], double localBestPerf[swarmsize], double localBestAge[swarmsize])
{
    int i;

    // If current performance of particle better than previous best, update previous best
    for (i = 0; i < swarmsize; i++)
    {
        if (perf[i] > localBestPerf[i])
        {
            copyParticle(localBest[i], swarm[i]);
            localBestPerf[i] = perf[i];
            localBestAge[i] = 1.0;
        }
    }
}


// Update the best performance of a particle neighborhood
void updateNBPerf(double localBest[swarmsize][datasize], double localBestPerf[swarmsize],
                  double nbBest[swarmsize][datasize], double nbBestPerf[swarmsize],
                  int neighbors[swarmsize][swarmsize])
{
    int i, j;
    int best_idx;

    // For each particle, check the best performances of its neighborhood
    for (i = 0; i < swarmsize; i++)
    {
        best_idx = i;
        for (j = 0; j < swarmsize; j++)
        {
            if (!neighbors[i][j])
                continue;

            // If current performance of particle better than previous best, update previous best
            if (localBestPerf[j] > localBestPerf[best_idx])
                best_idx = j;
        }
        nbBestPerf[i] = localBestPerf[best_idx];
        copyParticle(nbBest[i], localBest[best_idx]);
    }
}


// Copy one particle to another
void copyParticle(double particle1[datasize], double particle2[datasize])
{
    for (int i = 0; i < datasize; i++)
        particle1[i] = particle2[i];
}


// Find the best result found, set best to the particle, and return the performance
double bestInSwarm(double localBest[swarmsize][datasize], double localBestPerf[swarmsize], double best[datasize])
{
    int best_idx = argmax(localBestPerf, swarmsize);
    copyParticle(best, localBest[best_idx]);
    return localBestPerf[best_idx];
}



// MATH HELPER FUNCTIONS

// Find the argmax of the array
int argmax(double array[], int size)
{
    int best_idx = 0;
    for (int i = 1; i < size; i++)
    {
        if (array[i] > array[best_idx])
            best_idx = i;
    }
    return best_idx;
}


// Find the modulus of an integer
int mod(int num, int base)
{
    while (num >= base)
        num -= base;
    while (num < 0)
        num += base;
    return num;
}


// Generate random number in [vmin,vmax]
double randIn(double vmin, double vmax)
{
    return vmin + (vmax - vmin) * ((double)rand()) / ((double)RAND_MAX);
}


// S-function to transform v variable to [0,1]
double s(double v)
{
    if (v > 5)
        return 1.0;
    else if (v < -5)
        return 0.0;
    else
        return 1.0 / (1.0 + exp(-1 * v));
}