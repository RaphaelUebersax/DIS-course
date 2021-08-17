"""
/*****************************************************************************/
/* File:         test.py                                                 
/* Version:      3.0                                                         
/* Date:         6-June-21                                                   
/* Description:  This file contains results of simulation in Webots and computes
				 average and standard deviation of them. Those are then reported
				 in the final project report.          
/* Authors:      DIS group 12		                                            
/*****************************************************************************//
"""

import numpy as np

# Decide what results to print in consol
print_formation_oa = False
print_formation_crossing = False
print_flocking_oa = False
print_flocking_crossing = False





########## Obstacles world with Flock_0 and FORMATION #############################################

# Values correpsond to performance with migr : [5,0] [5,1] [5,-1] [5,2] [5,-2]
# Time of metrics starts a 0 and ends at 90 seconds

encoder_5_robots = np.array([0.764061, 0.783871, 0.787906, 0.764802, 0.789694])
acc_5_robots = np.array([0.577417, 0.744036, 0.580842, 0.728305, 0.751856])
gps_5_robots = np.array([0.691564,0.698840, 0.724569, 0.699651, 0.664240])
kalman_5_robots = np.array([0.749568, 0.783423, 0.769594, 0.759547, 0.788559])

encoder_7_robots = np.array([0.777433, 0.777680, 0.815993, 0.747160, 0.744792])
acc_7_robots = np.array([0.733322,0.691082, 0.764640, 0.709152, 0.757632])
gps_7_robots = np.array([0.729068, 0.700379, 0.724473, 0.673234,0.703257])
kalman_7_robots = np.array([0.782192, 0.776814, 0.816481, 0.748095, 0.783918])

encoder_9_robots = np.array([0.727295, 0.502209, 0.733788, 0.656434, 0.722633])
acc_9_robots = np.array([0.580659, 0.717611, 0.664648, 0.544267,0.634732])
gps_9_robots = np.array([0.606298,0.691092, 0.698967, 0.645809,0.690571])
kalman_9_robots = np.array([0.731966, 0.732398, 0.766617, 0.695754, 0.715029])

if print_formation_oa:
	print("Mean 5 robots: ",np.mean(encoder_5_robots)," ", np.mean(acc_5_robots)," ", np.mean(gps_5_robots)," ",np.mean(kalman_5_robots))
	print("Mean 7 robots: ",np.mean(encoder_7_robots)," ", np.mean(acc_7_robots)," ", np.mean(gps_7_robots)," ",np.mean(kalman_7_robots))
	print("Mean 9 robots: ",np.mean(encoder_9_robots)," ", np.mean(acc_9_robots)," ", np.mean(gps_9_robots)," ",np.mean(kalman_9_robots))

	print("std 5 robots: ",np.std(encoder_5_robots)," ", np.std(acc_5_robots)," ", np.std(gps_5_robots)," ",np.std(kalman_5_robots))
	print("std 7 robots: ",np.std(encoder_7_robots)," ", np.std(acc_7_robots)," ", np.std(gps_7_robots)," ",np.std(kalman_7_robots))
	print("std 9 robots: ",np.std(encoder_9_robots)," ", np.std(acc_9_robots)," ", np.std(gps_9_robots)," ",np.std(kalman_9_robots))




########### Crossing world with Flock_0 and Flock_1 and FORMATION ##################################

# Values correspond to performance when perpendicular (migr=[5,0]) corssing for [Flock0, Flock1]:
# Time of metrics starts a 0 and ends at 30 seconds

encoder = np.array([0.776, 0.770])
acc= np.array([0.750, 0.783])
gps = np.array([0.779, 0.736])
kalman = np.array([0.787, 0.789])

if print_formation_crossing:
	print("Crossing: ",encoder.tolist()," ", acc.tolist()," ", gps.tolist()," ",kalman.tolist())




###################################################################################################################################################


########## Obstacles world with Flock_0 and FLOCKING ################################################


# Values correpsond to performance with migr : [5,0] [5,1] [5,-1] [5,2] [5,-2]
# Time of metrics starts a 0 and ends at 90 seconds

encoder_5_robots_flock = np.array([0.607124, 0.371206, 0.600478, 0.552721, 0.540126])
acc_5_robots_flock = np.array([0.432216, 0.428883, 0.379197, 0.410591, 0.343401])
gps_5_robots_flock = np.array([0.470606, 0.386809, 0.493942,0.260787, 0.432344])
kalman_5_robots_flock = np.array([0.581553, 0.364340, 0.604823, 0.513955, 0.516263])

encoder_7_robots_flock = np.array([0.480404, 0.370931,0.569294,0.549542,0.356890])
acc_7_robots_flock = np.array([ 0.448151, 0.393943, 0.400318, 0.116876, 0.314478])
gps_7_robots_flock = np.array([0.437537, 0.442863, 0.450472, 0.453819,0.424685])
kalman_7_robots_flock = np.array([0.529055, 0.324731, 0.554106, 0.506048, 0.473102])

encoder_9_robots_flock = np.array([0.536278, 0.335594,0.465388, 0.255450,0.254926])
acc_9_robots_flock = np.array([0.390491, 0.186498, 0.184660,0.208416, 0.189676,])
gps_9_robots_flock = np.array([0.377483, 0.186442, 0.425778,0.130927,0.352496])
kalman_9_robots_flock = np.array([0.459499, 0.375248, 0.186959,0.439766,0.312983])


if print_flocking_oa:
	print("Mean 5 robots: ",np.mean(encoder_5_robots_flock)," ", np.mean(acc_5_robots_flock)," ", np.mean(gps_5_robots_flock)," ",np.mean(kalman_5_robots_flock))
	print("Mean 7 robots: ",np.mean(encoder_7_robots_flock)," ", np.mean(acc_7_robots_flock)," ", np.mean(gps_7_robots_flock)," ",np.mean(kalman_7_robots_flock))
	print("Mean 9 robots: ",np.mean(encoder_9_robots_flock)," ", np.mean(acc_9_robots_flock)," ", np.mean(gps_9_robots_flock)," ",np.mean(kalman_9_robots_flock))

	print("std 5 robots: ",np.std(encoder_5_robots_flock)," ", np.std(acc_5_robots_flock)," ", np.std(gps_5_robots_flock)," ",np.std(kalman_5_robots_flock))
	print("std 7 robots: ",np.std(encoder_7_robots_flock)," ", np.std(acc_7_robots_flock)," ", np.std(gps_7_robots_flock)," ",np.std(kalman_7_robots_flock))
	print("std 9 robots: ",np.std(encoder_9_robots_flock)," ", np.std(acc_9_robots_flock)," ", np.std(gps_9_robots_flock)," ",np.std(kalman_9_robots_flock))




########### Crossing world with Flock_0 and Flock_1 and FLOCKING ###################################

# Values correspond to performance when perpendicular (migr=[5,0]) corssing for [Flock0, Flock1]:
# Time of metrics starts a 0 and ends at 30 seconds

encoder_flock = np.array([0.433, 0.332])
acc_flock= np.array([0.345, 0.417])
gps_flock = np.array([0.375, 0.308])
kalman_flock = np.array([00.338, 0.312])

if print_flocking_crossing:
	print("Cossing: ",encoder_flock.tolist()," ", acc_flock.tolist()," ", gps_flock.tolist()," ",kalman_flock.tolist())
	