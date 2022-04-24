# **DRAFT**

## Particle Filter Project Writeup

### Video

Note: a full video localizing the robot through a larger section of the maze is located [here](videos/particle_filter_full.mp4), but only a shorter version could be embedded in the readme due to GitHub size constraints:

https://user-images.githubusercontent.com/38731359/164989913-9f8ae5e2-9b0f-4899-a982-924b5b336d76.mp4

### Team Members
Samir Rajani and Nick Auen

### Objectives
The goal of this project is to localize a robot within a space with a pre-determined map (drawn using SLAM) using the particle filter algorithm. The TurtleBot3 should be able to be dropped at any point throughout the maze and quickly determine its current location using a combination of odometry and LiDAR data; this location should be tracked as the robot moves throughout the maze.

### High Level Description
We solved the problem of robot localization using a particle filter approach. This involves initializing a particle cloud, moving particles according the movement of the robot (tracked via odometry), assigning probability weights to each of the particles according to the likelihood field model (based on LiDAR data), resampling the particle distribution based on these weights, and estimating the robot's pose by computing weighted averages of the particle distribution. As more odometry and LiDAR data is captured from the robot, particles cluster together, converging on the robot's true pose. It was also necessary account for noise in the robot's movement and sensor measurements. This is realized in the movement step, in which zero-centered Gaussian noise is added to the movement and rotation of particles, and in the measurement step, where we account for sensor noise by using a normal distribution to compute the probability of deviations from sensor measurements before multiplying probabilities into the weight.

### Main Steps
*cCode Location and Function Description*

1. Initialization of Particle Cloud
	> We initialize our particle cloud in the `initialize_particle_cloud` function. This function makes calls to two supporting functions, `normalize_particles` and `publish_particle_cloud`.
	
	> The function `initialize_particle_cloud` intializes particles in the particle cloud with random positions within the map, random orientations, and normalized weights. It first waits for the `frame_id` in the map header to be populated, indicating that the map has been loaded by the `get_map` callback. An array of possible particle locations is created based upon which spaces in the map are neither a wall nor unexplored. A random sample of these available locations is chosen, with replacement. For each particle, the coordinates of a chosen location, in pixels, is retrieved and converted to meters, a random yaw angle in the range 0-360 is assigned, and the weight is initialized to 1. A function call to `normalize_particles` is then made, which divides each particle's weight by the sum of all the weights, ensuring the particle weights sum to 1. Finally, a call to `rospy.sleep()` ensures the publisher has had time to set up, and the particle cloud is published in `publish_particle_cloud`. 

2.  Movement Model
	> The movement model is implemented in the `update_particles_with_motion_model` function, which is called within `robot_scan_received`. Calls are also made to the helper functions `get_yaw_from_pose` and `sample_normal_distribution`.

	> The `update_particles_with_motion_model` function takes the x, y, and yaw values from the robot's odometry and compares them with the previous timestep's odometry data. Odometry data is updated after a minimum threshold for linear and/or angular movement has been reached; these thresholds are initialized within the `__init__` function for the `ParticleFilter` class. The robot's movement is modeled as a rotation, followed by a translation, followed by another rotation; the angle of the first rotation is computed by taking the arctan of the ratio of the y displacement to the x displacement of the robot, the magnitude of the translation is computed using the Euclidean distance, and the angle of the second rotation is computed by subtracting the previous yaw and first rotation from the current yaw. We the create a new particle cloud and iterate over each particle in the original particle cloud, applying the rotations and translation, with additional Gaussian noise added through a call to `sample_normal_distribution` (the width of the distribution is computed as a linear combination of the rotation and translation values, using noise parameters `alpha_1`, `alpha_2`, `alpha_3`, and `alpha_4`). The implementation of the `sample_normal_distribution` function is derived from Probabilistic Robotics; it approximates Gaussian noise by summing randomly generated numbers between -1 and 1.

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
