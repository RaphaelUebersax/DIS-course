#define DATASIZE 4  // if flocking then... if formation them...
#define SWARMSIZE 10 

// Functions
void pso(int n_swarmsize, int n_nb, double lweight, double nbweight, double vmax, double min, double max, int iterations, int n_datasize, double (*fitness_ptr)(double[]), double best[]);
void findPerformance(double[][DATASIZE], double[], double[], char, int[][SWARMSIZE]);            // Find the current performance of the swarm
void updateLocalPerf(double[][DATASIZE], double[], double[][DATASIZE], double[], double[]);      // Update the best performance of a single particle
void updateNBPerf(double[][DATASIZE], double[], double[][DATASIZE], double[], int[][SWARMSIZE]); // Update the best performance of a particle neighborhood
void copyParticle(double[], double[]);                                                           // Copy value of one particle to another
double bestInSwarm(double[][DATASIZE], double[], double[]);                                      // Find the best result in a swarm
int argmax(double[], int);                                                                       // Find index of maximal value
int mod(int, int);                                                                               // Modulus function
double randIn(double, double);                                                                   // Generate random number in [vmin, vmax]
double s(double);                                                                                // S-function to transform [-infinity,infinity] to [0,1]
