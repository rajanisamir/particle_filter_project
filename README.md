# particle_filter_project

## Implementation Plan

**Team Members:** Samir Rajani, Nick Auen

**Descriptions of Planned Component Implementations:**
- How will you initialize your particle cloud?
  - We will initialize the particle cloud by selecting points at random (and at random orientations) from within the map. If the maze is perfectly rectangular (i.e. has no gaps), we can do this by independently selecting a random value for both x and y, for each particle we generate (and a value between 0 and 360 degrees for the orientation).
  - The initialization of the particle cloud can be tested by plotting the locations of each particle on the map and examining whether the locations appear uniformly distributed. Once other components are completed, we can also ensure that the number of particles generated is high enough for the particle clouds to converge on the robot's position.
- How will you update the positions of the particles based on the movements of the robot?
  - We will update the positions of the particles by using the robot's odometry to figure out the changes to the robot's position and orientation, and then by adding these changes to the respective components of each particle. We can incoporate Gaussian noise by instead constructing a Gaussian distribution centered at the theoretical change to position and orientation, and then by sampling multiple times from this distribution to add the changes.
  - We will test this component by moving the robot and displaying the locations of the particles on the map, qualitatively ensuring that the particles move and rotate roughly with the robot.
- How will you compute importance weights of each particle after receiving the robot's laser scan data?
  - Importance weights of each particle will be computed by comparing the laser scan of the robot with the theoretical laser scan of each particle using a formula that reflects the similarity of the scans.
  - We will test the importance weight computation by displaying the weights of each particle in the initial (random) distribution and ensuring that the largest weights occur at locations that are either near the robot or which would contain similar laser scans.
- How will you normalize the particles' importance weights and resample the particles?
  - The weights will be normalized by dividing each particle's weight by the sum of the weights. Particles can be resampled with replacement by using a function like `random.choice` from numpy, or by writing a function to do so manually (for example, by assigning each particle an interval corresponding with its weight through a cumulative sum of weights, generating a random value between 0 and 1, and figuring out which interval corresponds with the generated number through a binary search).
  - We will test the normalization by ensuring that the particle weights sum to one. Resampling can be tested by checking to see if the areas with the highest importance weights from the previous step were selected most frequently when resampling.
- How will you update the estimated pose of the robot?
  - The estimated pose of the robot can be updated by computing the component-wise average of the positions and orientations of each of the resampled particles.
  - We will test the estimated pose by navigating our robot through the maze and examining the pose to see if, after the particles converge, it reflects the robot's true position. We can also display the current pose and particle positions and orientations on a map, examining qualitatively to see if they match up.
- How will you incorporate noise into your particle filter localization?
  - We will generate artificial noise by sampling from a normal distribution centered around a theoretical value to produce Gaussian noise. For example, we will account for noise in the motors by adding Gaussian noise to the state transition distribution.
  - We will test the incorporation of noise by comparing the robot's localization with and without noise, varying the noise parameters (such as the width of the Gaussian) to find the most appropriate value. 
 
**Timeline:** We aim to have components implemented by the following times:
  - Initialization of particle cloud: Thursday, April 14th
  - Updating position of particles: Saturday, April 16th
  - Importance weight computation: Monday, April 18th
  - Normalization of weights and resampling: Tuesday, April 19th
  - Updating robot pose: Wednesday, April 20th
  - Incorporation of noise: Thurdsay, April 21st

We will use the remaining time to perform further tests of our implementation and to tie up any loose ends.
