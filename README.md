# **DRAFT**



# particle_localization_project readme

## Particle Localization Project Writeup

### Video

Note: a full video is located [here](videos/particle_filter_full.mp4), but only a smaller version could be embedded in the readme.

![particle_filter](videos/particle_filter_shortened.mp4)

### Objectives
The goal of this project is to create a method of robot localization by means of using the particle filter algorithm within a pre-mapped space. The turtlebot3 should be able to be dropped at any point throughout the maze and quickly determine its current location using a combination of odometry and LiDAR data, after which it should be able to navigate to the exit of the maze. 

### High Level Description
In order to create a robust robot localization solution, these main components were utilized: Creating a particle cloud of 1,000 particles, each of which act as a guess for the pose of the robot (x, y, yaw). Odometry and LiDAR data (with gaussian error included) from the robot is then incorporated into the weights (probabilities) of the particles. Based on the updated probabilities, the particles are resampled and become increasingly representative of the current estimation of the robot's pose. As more odometry and LiDAR data is captured from the robot, particles cluster together close to the robot's true pose. 

### Main Steps
*Include code location and function/code description*

1. Initialization of particle cloud
	> This code is in the *initialize_particle_cloud* function and is also supported by the *normalize_particles* and *publish_particle_cloud* functions. 
	
	>The function waits for the frame_id to be populated (allowing for the pre-determined map to be loaded into place). Based on the measurements of the map (width and height), an array of possible particle locations (indices) is created, each with empty data inside of them. From these, a random sample based on the number of particles (1000) are randomly chosen. For each particle in this array of chosen_indices, an x, y, and yaw angle (0-360) is assigned, with the weights of each particle initialized to 1. In *normalize_particles*, the weights are then normalized to all sum to 1 to allow for them to act as probabilities in future resampling. *publish_particle_cloud* then publishes this array with each particle containing an x, y, and yaw angle within the boundaries of the map.
2.  Movement model
	> This code is in the *update_particles_with_motion_model* function. This function is then called within the *robot_scan_received* function in order to update the particles with this motion model.

	> This function takes the x, y, and current yaw from the robot's odometry and compares it with the previous timestep's odometry data (if available). Odometry data is updated after a minimum threshold for linear and/or angular movement has been reached - these values are defined within the *__init__* function (linear = 0.2 & ang = pi/6). The change in rotation of the robot is calculated using the math.atan2 function and the change in 2D space is calculated by finding the Euclidean distance between the previous and current time-step x and y points. These changes are then added to the current x, y, and theta's of each particle in the particle cloud, thus updating them according to the robot's odometry data. 
3.  Measurement model
	> This code is in the *update-particle_weights_with_measurement_model* function which also calls the *likelihood_field.py* script.
	
	> This function cycles through each point in the cloud and uses their x, y, and theta values with the *get_closest_obstacle_distance* function to determine what the measurement value from their LiDAR Scanner would be. This distance is then computed into a probability using a Gaussian distribution centered at zero. 
4.  Resampling
	> This code is in the *resample_partcles* function and is called in the *robot_scan_received* function.

	> This function simply redraws the particles (with replacement) randomly according to their assigned weights by using the *draw_random_sample* function with the particle cloud, the particles, and their respective weights as parameters.
5.  Incorporation of noise
	> Noise is incorporated in both the *update_particles_with_motion_model* and *update_particles_weights_with_measurement_model* functions. 
	> In the *update_particles_with_motion_model* function, noise is incorporated into the x, y, and yaw values by **insert**
6.  Updating estimated robot pose
	> This code is in the *update_estimated_robot_pose* function and is called in the *robot_scanned_received* function.

	> This function first computes the weighted sum for all the particles and then divides the average x, y, and theta values by this weighted sum. These weighted means are then converted into Point and Quaternion objects and are used to update the estimated robot pose.
8.  Optimization of parameters

### Challenges

### Future Work

### Takeaways 

----------------------
## Implementation Plan

**Team Members:** Samir Rajani, Nick Auen

**Descriptions of Planned Component Implementations:**
- How will you initialize your particle cloud?
  - We will initialize the particle cloud by selecting points at random (and at random orientations) from within the map. If the maze is perfectly rectangular (i.e. has no gaps), we can do this by independently selecting a random value for both x and y, for each particle we generate (and a value between 0 and 360 degrees for the orientation). Importantly, we will exclude those values whch would place initial particles within walls or other boundaries in the maze.
  - The initialization of the particle cloud can be tested by plotting the locations of each particle on the map and examining whether the locations appear uniformly distributed. Once other components are implemented, we can also ensure that the number of particles generated is high enough for the particle clouds to converge on the robot's position.
- How will you update the positions of the particles based on the movements of the robot?
  - We will update the positions of the particles by using the robot's odometry to figure out the changes to the robot's position and orientation, and then by adding these changes to the respective components of each particle. We can incoporate Gaussian noise by  constructing a Gaussian distribution centered at the theoretical change to position and orientation, and then by sampling multiple times from this distribution to add the changes.
  - We will test this component by moving the robot and displaying the locations of the particles on the map, qualitatively ensuring that the particles move and rotate roughly with the robot and also incorpoate the randomly sampled noise.
- How will you compute importance weights of each particle after receiving the robot's laser scan data?
  - Importance weights of each particle will be computed by comparing the laser scan of the robot with the theoretical laser scan of each particle using a formula that reflects the similarity of the scans. In particular, we can use the likelihood field range finder model covered in Lecture 6.
  - We will test the importance weight computation by displaying the weights of each particle in the initial (random) distribution and ensuring that the largest weights occur at locations that are either near the robot or which would produce similar laser scans.
- How will you normalize the particles' importance weights and resample the particles?
  - The weights will be normalized by dividing each particle's weight by the sum of the weights. Particles can be resampled with replacement by using a function like `random.choice` from numpy, or by writing a function to do so manually (for example, by assigning each particle an interval corresponding with its weight through a cumulative sum of weights, generating a random value between 0 and 1, and figuring out which interval corresponds with the generated number through a binary search).
  - We will test the normalization by ensuring that the particle weights sum to one. Resampling can be tested by checking to see if the areas with the highest importance weights from the previous step were selected most frequently when resampling.
- How will you update the estimated pose of the robot?
  - The estimated pose of the robot can be updated by computing the component-wise average of the positions and orientations of each of the resampled particles.
  - We will test the estimated pose by navigating our robot through the maze and examining the pose to see if, after the particles converge, it reflects the robot's true position. We can also display the current pose and particle positions and orientations on a map, examining qualitatively to see if they match up.
- How will you incorporate noise into your particle filter localization?
  - We will generate artificial noise by sampling from a normal distribution centered around a theoretical value to produce Gaussian noise. For example, we will account for noise in the motors by adding Gaussian noise to the state transition distribution, and we will account for measurement noise by adding a normal distribution to the computation of the hit probability in the beam model.
  - We will test the incorporation of noise by comparing the robot's localization with and without noise, varying the noise parameters (such as the width of the Gaussian) to find the most appropriate value. 
 
**Timeline:** We aim to have components implemented by the following times:
  - Initialization of particle cloud: Thursday, April 14th
  - Updating position of particles: Saturday, April 16th
  - Importance weight computation: Monday, April 18th
  - Normalization of weights and resampling: Tuesday, April 19th
  - Updating robot pose: Wednesday, April 20th
  - Incorporation of noise: Thurdsay, April 21st

We will use the remaining time to perform further tests of our implementation and to tie up any loose ends.
