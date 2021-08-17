
# Multi-robot Navigation in Cluttered and Dynamic Environments

CS-466 Distributed Intelligent Systems Project, Spring 2021, Group 12

Group members: Raphael Uebersax, Jonas Perolini, Deblina Bhattacharjee, and Utku Norman

In this code, the localisation, flocking and formation of a multi-robot system is studied, when navigating cluttered or dynamic environments. The robots navigate two environment scenarios to reach their goal, namely obstacle avoidance (i.e. *obstacles*) and *crossing*. The experiments are conducted on the Webots platform and are tested on multiple benchmark test worlds. Further, an off-the-shelf metaheuristic algorithm, Particle Swarm Optimisation (PSO), was used to tune the hyperparameters of both the flocking and formation strategies. With this code, we do comparative analysis of the best performance (average error) across all the navigation strategies, including localisation, flocking and formation.


## Structure <a name="content"></a>

The code is structure into three project folders, where each contain the subdirectories controllers/ and worlds/, and are dedicated for the following purposes:

1. [main](main/): the main controllers and worlds with fixed parameters that we used to obtain the results like testing for localisation performance in our report
2. [pso](pso/): PSO-related controllers and the worlds where we run PSO to fine-tune the hyperparameters, e.g. the weights of the flocking and formation algorithm
3. [test](test/): the test world and the controllers from main configured according to the test worlds

Regarding the naming conventions, we differentiate between two controllers: Flock_0 and Flock_1, where the 0 and 1 refer to the first and second group of robots, respectively, so that specifically in the crossing scenario, the two teams can seperate their teammates from the rest. The related supervisor controllers are named with a prefix "super_".

Regarding the configurations, the relevant hyperparameters to set while running a controller or a supervisor's controller are given with the define directives on the top of the controller, with the corresponding description on what it means and the available values. For instance, At the top of each controller e.g. Flock_1, we set FLOCKING and FORMATION depending on the algorithm to test: e.g., FLOCKING=true and FORMATION=false for testing the flocking algorithm, and vice versa.

For the results, the performance values for the different localisation methods and number of robots for the flocking and formation algorithms are given in [main/controllers/Part_2_results.py](main/controllers/Part_2_results.py), that computes their mean and standard deviation.
For the test worlds, performance values for the localisation by Kalman filtering, for the flocking and formation algorithms are in [test/reporting.py](test/reporting.py), that also computes their mean and standard deviation.

## How to run

1. For localisation performance assessment, use the controller [localization](main/controllers/localization) with supervisor [super_localization](main/controllers/super_localization) to obtain the average localisation accuracy for different localisation methods over the two trajectories. 

	To set the trajectory, modify the constant TRAJECTORY_NO in [localization/localization.c](main/controllers/localization/localization.c), by setting it to 1 or 2 to run the trajectory 1 or trajectory 2 respectively.

2. For evaluation of the flocking and formation algorithms in the obstacles scenario, use the controller [Flock_0](main/controllers/Flock_0).

	Set the constant WORLD to 0. (0: OBSTACLES, 1: CROSSING) in [Flock_0/Flock_0.c](main/controllers/Flock_0/Flock_0.c).

	Set the constants FLOCKING=true and FORMATION=false in [Flock_0/Flock_0.c](main/controllers/Flock_0/Flock_0.c) to run the flocking-related algorithms, and vice versa for the formation-related algorithms. 
	For the supervisors, use [super_flocking](main/controllers/super_flocking/) to compute the average flocking metric, and [super_formation](main/controllers/super_formation/) to compute the average formation metric.

	To enable a specific localisation method, set the constants ENCODER_LOC, ODOMETRY_ACC, GPS_LOC and ACTIVATE_KALMAN accordingly. In particular, for localisation by Kalman filter, ACTIVATE_KALMAN=true and ENCODER_LOC=false, ODOMETRY_ACC=False and GPS_LOC=false. For localisation by encoders, ENCODER_LOC=true and the others false. For localisation by GPS only, GPS_LOC=true and the others false. For localisation by accelerometers (with the heading from encoders), ODOMETRY_ACC=true and the others false. 

	For formation, set the migratory urge by commenting out the desired urge along x and y, with the variable "migr" in [Flock_0/Flock_0.c](main/controllers/Flock_0/Flock_0.c), among the five different urge values that we have tested. 

3. For evaluation of the flocking and formation algorithms in the crossing scenario, use the controllers [Flock_0](main/controllers/Flock_0) and [Flock_1](main/controllers/Flock_1) for the groups 1 and group 2, respectively.

	Set the constant WORLD to 1. (0: OBSTACLES, 1: CROSSING) in [Flock_0/Flock_0.c](main/controllers/Flock_0/Flock_0.c) and [Flock_1/Flock_1.c](main/controllers/Flock_1/Flock_1.c).

	As in the previous item, set the constants FLOCKING=true and FORMATION=false in [Flock_0/Flock_0.c](main/controllers/Flock_0/Flock_0.c) and [Flock_1/Flock_1.c](main/controllers/Flock_1/Flock_1.c) to run the flocking-related algorithms, and vice versa for the formation-related algorithms. 

	For the supervisors, use [super_flocking](main/controllers/super_flocking/) to compute the average flocking metric, or [super_formation](main/controllers/super_formation_1/) to compute the average formation metric for the first group. For the second group along with the first group, also use [super_flocking_1](main/controllers/super_flocking_1/) or [super_formation_1](main/controllers/super_formation_1/) as a second supervisor.

4. For PSO, use the supervisor [pso_supervisor](pso/controllers/super_flocking_1/) with the controller s[Flock_0](pso/controllers/Flock_0) and [Flock_1](pso/controllers/Flock_1) according to the obstacles and crossing world as described before.
