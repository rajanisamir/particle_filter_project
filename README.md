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
- How will you normalize the particles' importance weights and resample the particles?
- How will you update the estimated pose of the robot?
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
