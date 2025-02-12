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
*Code Location and Function Description*

1. Initialization of Particle Cloud
	> We initialize our particle cloud in the `initialize_particle_cloud` function. This function makes calls to two supporting functions, `normalize_particles` and `publish_particle_cloud`.
	
	> The function `initialize_particle_cloud` intializes particles in the particle cloud with random positions within the map, random orientations, and normalized weights. It first waits for the `frame_id` in the map header to be populated, indicating that the map has been loaded by the `get_map` callback. An array of possible particle locations is created based upon which spaces in the map are neither a wall nor unexplored. A random sample of these available locations is chosen, with replacement. For each particle, the coordinates of a chosen location, in pixels, is retrieved and converted to meters, a random yaw angle in the range 0-360 is assigned, and the weight is initialized to 1. A function call to `normalize_particles` is then made, which divides each particle's weight by the sum of all the weights, ensuring the particle weights sum to 1. Finally, a call to `rospy.sleep()` ensures the publisher has had time to set up, and the particle cloud is published in `publish_particle_cloud`. 

2.  Movement Model
	> The movement model is implemented in the `update_particles_with_motion_model` function, which is called within `robot_scan_received`. Calls are also made to the helper functions `get_yaw_from_pose` and `sample_normal_distribution`.

	> The `update_particles_with_motion_model` function takes the x, y, and yaw values from the robot's odometry and compares them with the previous timestep's odometry data. Odometry data is updated after a minimum threshold for linear and/or angular movement has been reached; these thresholds are initialized within the `__init__` function for the `ParticleFilter` class. The robot's movement is modeled as a rotation, followed by a translation, followed by another rotation; the angle of the first rotation is computed by taking the arctan of the ratio of the y displacement to the x displacement of the robot, the magnitude of the translation is computed using the Euclidean distance, and the angle of the second rotation is computed by subtracting the previous yaw and first rotation from the current yaw. We the create a new particle cloud and iterate over each particle in the original particle cloud, applying the rotations and translation, with additional Gaussian noise added through a call to `sample_normal_distribution` (the width of the distribution is computed as a linear combination of the rotation and translation values, using noise parameters `alpha_1`, `alpha_2`, `alpha_3`, and `alpha_4`). The implementation of the `sample_normal_distribution` function is derived from Probabilistic Robotics; it approximates Gaussian noise by summing randomly generated numbers between -1 and 1.

https://user-images.githubusercontent.com/38731359/165220535-84e84041-11ac-4a53-ad5c-c7612c81cc08.mp4

3.  Measurement Model
	> The measurement model is implemented in the `update_particle_weights_with_measurement_model` function, which also calls the `get_closest_obstacle_distance` function from likelihood_field.py.
	
	> The function `update_particle_weights_with_measurement_model` computes importance weights for each particle using the likelihood field measurement algorithm. For each particle in the cloud, it checks the LiDAR measurements of the robot in all 360 degrees. If the robot detects an object at a particular angle, the theoretical position of that object relative to the particle is computed, and the closest object to that location is calculated from the map in `get_closest_obstacle_distance`. The probability of measuring this distance is computed based on a zero-centered Gaussian distribution with width `likelihood_sigma`, which is initialized in the `__init__` function for the `ParticleFilter` class, and the probability is multiplied into the weight. 
	
4.  Resampling
	> Resampling is performed in the `resample_particles` function, which makes an additional call to the helper function `draw_random_sample`.

	> The `resample_particles` function first forms a list of the particle weights to use as probabilities for the sampling function. It then sets the particle cloud to a random sample of the current particles, according to the probability distribution specified by their weights. This is achieved through a call to `draw_random_sample`, which uses `np.random.choice` to draw a random sample from a specified list of elements and probabilities, with replacement.

5.  Incorporation of Noise
	> Noise is incorporated in the `update_particles_with_motion_model` and `update_particle_weights_with_measurement_model` functions. 

	> In the `update_particles_with_motion_model` function, noise is incorporated by sampling from a normal distribution when translating and rotating the particles along with the robot's movement. The implementation of the `sample_normal_distribution` function is derived from Probabilistic Robotics; it approximates Gaussian noise by summing randomly generated numbers between -1 and 1. The width of the distribution is computed as a linear combination of the rotation and translation values, using noise parameters `alpha_1`, `alpha_2`, `alpha_3`, and `alpha_4`. Thus, if the robot deviates from the path computed from its odometry, the movement will quickly be corrected by particles whose noise aligns with the deviation. In the `update_particle_weights_with_measurement_model` function, we account for noise in the sensors by allowing the theoretical positions of objects to deviate slightly from the robot's LiDAR measurements without significantly decreasing the weight of a particle. In particular, for each sensor measurement, the weight of a particle is multiplied by the probability of measuring a particular distance to the nearest object, from where the robot actually detects an object. This proability is computed in `compute_prob_zero_centered_gaussian`, with a finite width specified by `likelihood_sigma`.

6.  Updating Estimated Robot Pose
	> We update the estimated robot pose in the `update_estimated_robot_pose` function, which uses `get_yaw_from_pose` as a helper function.

	> The `update_estimated_robot_pose` function computes the sum of the weights of the particles, and then computes the weighted mean of the x and y coordinates of the particles using this sum. To compute the angle of the estimated pose, the angle is split into its x- and y- components, of which weighted means are computed separately, before computing the angle associated with these averaged components using arctan. A point and quaternion are constructed with the averaged coordinates and rotation, and the robot pose is updated.

7.  Optimization of Parameters
	> Control over the parameters associated with the particle filter are located in the `__init__` function for the ParticleFilter class; the relevant parameters are `num_particles`, `lin_mvmt_threshold`, `ang_mvmt_threshold`, `alpha_1`, `alpha_2`, `alpha_3`, `alpha_4`, and `likelihood_sigma`.

	> The number of particles is associated with a tradeoff between particle filter precision and computation time. In practice, we found that having less than 500 particles could result in the particle cloud converging on an inaccurate location, either because of statistical uncertainties associated with resampling or because the initial particle cloud did not contain enough particles representative of the robot's location. Meanwhile, having more than 2000 particles resulted in the computation falling far behind the robot's movement (but it should be noted that this depends on the machine running the code). Thus, 1000 particles was chosen as a solid middle ground between these two issues. The linear and angular movement threshold parameters control the amount the robot must move before we perform an update (including the movement update, measurement update, etc.); the default values of 0.2 and π/6 were found to be appropriate for the physical robot. The parameters `alpha_1`, `alpha_2`, `alpha_3`, and `alpha_4` the amount of noise added to the movement update step; in particular, they are used to form a linear combination of the two rotations and translation, and this linear combination is passed as a width to the `sample_normal_distribution` function. Values of 0.3 were found to work well for each of these parameters, because they permitted large amounts of noise in the robot's movement. Finally, the `likelihood_sigma` controls the width of the Gaussian in the measurement update used to compute the probability of measuring a particular distance to the nearest object, from where the robot actually detects an object. This probability is computed in `compute_prob_zero_centered_gaussian`, and is multiplied into the weight in `update_particle_weights_with_measurement_model`. A value of 0.5 was found to provide a good balance between being overly restrictive in particle selection (and thus risking honing in on the wrong area of the map) and being overly lenient (and thus never converging on a particular location). It should also be noted that the number of sensor measurements might be considered a parameter that can be varied, but we found that using all 360 degrees of measurement did not significantly hinder performance with our particle number.

### Challenges
One of the main challenges we faced was that the particles didn't form a wide enough cluster, even when we increased the amount of noise in the movement step. We eventually realized that the issue was that, even though particles spread out in the movement step, the width of the Gaussian that we used to compute the probabilities of sensor measurements in the measurement step was too small, and the particles that deviated too far from the sensor measurements were restricted. This wasn't ideal, because a large amount of sensor noise in a particular area of the maze could result in particles converging on the wrong area. We solved this issue by increasing the value of the `likelihood_sigma` parameter from 0.1 to 0.5. We also ran into an issue in which computations at each step took too long, and an error related to using data with old timestamps prevented the code from running. We solved this by reducing the number of particles to 1000, which sped up computations. Finally, we were able to mitigate slow computation times by reducing the robot's speed, which allowed for more time to perform computations before the next linear or angular movement threshold was reached.

### Future Work
One way in which the particle filter localization could be improved is by adding in values Z_rand and Z_max, which would allow for deviations in sensor measurements by introducing expected probabilities of random sensor measurements and failure to detect obstacles. We could improve the map formed with SLAM by applying manual corrections to the .pgm file, such as by connecting some of the walls that got misaligned, as seen in the top right of the image below:

<img width="289" alt="Map" src="https://user-images.githubusercontent.com/38731359/165220586-168f71ef-f6be-4e4c-94d7-31937ed702fb.png">

We could also try to eliminate cases in which the particles don't converge to the position of the robot by reintroducing some particles after each resampling, but this issue is typically solved by increasing the number of particles from the beginning. Finally, if we wanted to have the robot actually navigate out of the maze, we could implement a path planning algorithm such as A* search to find an optimal route.

### Takeaways 
- Planning out components and testing them individually is essential. When we completed the implementation plan for this project, we provided a way of testing each major component of the project. This list proved very useful, because it was much easier to pinpoint an issue when something came up. For example, if we had already tested the movement model, and we just implemented the measurement model on top of it, an ill-behaved test was likely to come from the measurement model.
- Tweaking parameters can take a surprising amount of time, and communicating with your collaborators about what worked best during testing is necessary, because it avoids time wasted by tweaking the same values repeatedly. We found that finding an accurate value for our `likelihood_sigma` parameter took a while, because we didn't have great intuition about how modifying the width of a particular Gaussian in the measurement model would affect the distribution of particles. It should also be noted that testing parameters is made much more complicated when the parameters interact with each other, so writing down the results of testing is useful when trying to make sense of how several different parameters behave in combination.

----------------------
## Implementation Plan

**Team Members:** Samir Rajani, Nick Auen

**Descriptions of Planned Component Implementations:**
- How will you initialize your particle cloud?
  - We will initialize the particle cloud by selecting points at random (and at random orientations) from within the map. If the maze is perfectly rectangular (i.e. has no gaps), we can do this by independently selecting a random value for both x and y, for each particle we generate (and a value between 0 and 360 degrees for the orientation). Importantly, we will exclude those values whch would place initial particles within walls or other boundaries in the maze.
  - The initialization of the particle cloud can be tested by plotting the locations of each particle on the map and examining whether the locations appear uniformly distributed. Once other components are implemented, we can also ensure that the number of particles generated is high enough for the particle clouds to converge on the robot's position.
- How will you update the positions of the particles based on the movements of the robot?
  - We will update the positions of the particles by using the robot's odometry to figure out the changes to the robot's position and orientation, and then by adding these changes to the respective components of each particle. We can incoporate Gaussian noise by constructing a Gaussian distribution centered at the theoretical change to position and orientation, and then by sampling multiple times from this distribution to add the changes.
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
