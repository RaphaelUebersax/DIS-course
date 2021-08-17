"""
/*****************************************************************************/
/* File:         test.py                                                 
/* Version:      3.0                                                         
/* Date:         6-June-21                                                   
/* Description:  This file contains results of simulation in Webots and computes
				 average and standard deviation of them. Those are then reported
				 in the final project report. For the test worlds.         
/* Authors:      DIS group 12		                                            
/*****************************************************************************//
"""

import numpy as np

########## Obstacles world with Flock_0 and FORMATION #############################################

# Values correpsond to performance with migr : [5,0] [5,1] [5,-1] [5,2] [5,-2]
# Time of metrics starts a 0 and ends at 90 seconds
kalman_5_robots = np.array([0.713650, 0.742241, 0.767135, 0.754013, 0.764948])

print("Formation in obstacles with Kalman: mean: ", np.mean(
    kalman_5_robots), " std: ", np.std(kalman_5_robots))

########### Crossing world with Flock_0 and Flock_1 and FORMATION ##################################

# Values correspond to performance when perpendicular (migr=[5,0]) corssing for [Flock0, Flock1]:
# Time of metrics starts a 0 and ends at 30 seconds

kalman = np.array([0.769784, 0.752149])

print("Formation in crossing with Kalman: ", kalman.tolist())


###################################################################################################################################################


########## Obstacles world with Flock_0 and FLOCKING ################################################


# Values correpsond to performance with migr : [5,0] [5,1] [5,-1] [5,2] [5,-2]
# Time of metrics starts a 0 and ends at 90 seconds

kalman_5_robots_flock = np.array([0.246281, 0.278616, 0.404573, 0.444752, 0.456839])


print("Flocking in obstacles with Kalman: mean: ", np.mean(kalman_5_robots_flock),
      " std: ", np.std(kalman_5_robots_flock))


########### Crossing world with Flock_0 and Flock_1 and FLOCKING ###################################

# Values correspond to performance when perpendicular (migr=[5,0]) crossing for [Flock0, Flock1]:
# Time of metrics starts a 0 and ends at 30 seconds

# kalman_flock = np.array([0.338, 0.312])
kalman_flock = np.array([0.519556, 0.532522])

print("Flocking in crossing with kalman: ",  kalman_flock.tolist())
