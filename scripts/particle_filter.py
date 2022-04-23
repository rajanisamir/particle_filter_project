#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
from numpy.random import random_sample
import math

from likelihood_field import LikelihoodField

from random import randint, random, sample


def sample_normal_distribution(b):
    """ A helper function that samples from a normal distirbution with variance b.
        Implementation taken from Probabilistic Robotics. """
    return (b / 6) * sum([-1 + 2 * np.random.random() for i in range(12)])


def compute_prob_zero_centered_gaussian(dist, sd):
    """ A helper function from the likelihood field class exercise that takes in
        distance from zero (dist) and standard deviation (sd) for gaussian and
        returns probability (likelihood) of observation """
    c = 1.0 / (sd * math.sqrt(2 * math.pi))
    prob = c * math.exp((-math.pow(dist,2))/(2 * math.pow(sd, 2)))
    return prob


def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw """

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw


def draw_random_sample(sample_list, n, probabilities):
    """ A helper function that draws a random sample of n elements from a given list of
    choices and their specified probabilities """
    return np.random.choice(sample_list, size=n, replace=True, p=probabilities).tolist()


class Particle:

    def __init__(self, pose, w):
        """ Constructor that initializes particle with pose and weight. """

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w


class ParticleFilter:

    def __init__(self):
        """ Constructor that intializes particle filter by the node, publishers, subscribers, and the particle cloud. """

        # once everything is setup initialized will be set to true
        self.initialized = False        

        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map and likelihood field
        self.map = OccupancyGrid()

        # the number of particles used in the particle filter
        self.num_particles = 400

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we perform an update
        self.lin_mvmt_threshold = 0.2        
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None

        # set up noise parameters
        self.alpha_1 = 0.3
        self.alpha_2 = 0.3
        self.alpha_3 = 0.3
        self.alpha_4 = 0.3

        # Set up publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.initialized = True


    def get_map(self, data):
        """Initializes the map and likelihood field for the particle filter. """

        self.map = data
        self.likelihood_field = LikelihoodField(map=self.map)
    

    def initialize_particle_cloud(self):
        """ Initializes the particles in the particle cloud with random positions within the map,
        random orientations, and normalized weights. """
        
        # wait until get_map callback sets the map by checking if frame_id has been populated
        while self.map.header.frame_id == '':
            continue

        # store list of all open spaces in map in available_indices
        available_indices = []

        # iterate through all pixels in the map
        for i in range(self.map.info.width * self.map.info.height):
            # if map pixel is empty, append it to available_indices
            if self.map.data[i] == 0:
                available_indices.append(i)

        # choose particle positions uniformly at random, with replacement, from available_indices
        chosen_indices = draw_random_sample(available_indices, self.num_particles, None)
        
        # for each chosen particle index, initialize a particle.
        for idx in chosen_indices:
            # retrieve x and y coordinates from row-major ordering; convert from px to m using
            #   resolution; choose a random yaw angle in the range [0, 360)
            idx_y = int(idx / self.map.info.width)
            idx_x = idx % self.map.info.width
            y = idx_y * self.map.info.resolution + self.map.info.origin.position.y
            x = idx_x * self.map.info.resolution + self.map.info.origin.position.x
            theta = np.random.random() * 360

            # initialize point and quaternion for particle pose
            point = Point(x, y, 0.)
            quaternion = Quaternion(*quaternion_from_euler(0., 0., theta, 'rxyz'))

            # initialize particle and append to particle cloud; weights are initialized to equal
            #   values, and they are normalized later
            particle_pose = Pose(point, quaternion)
            particle_weight = 1
            particle = Particle(particle_pose, particle_weight)
            self.particle_cloud.append(particle)
        
        # normalize particle weights
        self.normalize_particles()

        # give publisher time to set up, and publish particle cloud
        rospy.sleep(3)
        self.publish_particle_cloud()


    def normalize_particles(self):
        """ Makes all the particle weights sum to 1.0. """
        
        # compute sum of particle weights
        weight_sum = sum(particle.w for particle in self.particle_cloud)

        # divide each particle's weight by the sum of all particle weights
        for particle in self.particle_cloud:
            particle.w /= weight_sum
        

    def publish_particle_cloud(self):
        """ Constructs a pose array from the particle cloud and publishes it using self.particles_pub. """

        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)

        # construct pose array from particle cloud
        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)


    def publish_estimated_robot_pose(self):
        """ Publishes the estimated pose of the robot (with a timestamp) using self.robot_estimate_pub."""

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)


    def resample_particles(self):
        """ Resamples particles, with replacement, according to their weights. """

        # get list of particle weights
        weights = [particle.w for particle in self.particle_cloud]

        # set self.particle_cloud to a random sample of particles, with probabilities corresponding with particle weights
        self.particle_cloud = draw_random_sample(self.particle_cloud, self.num_particles, weights)


    def robot_scan_received(self, data):
        """ Carry out the logic of the particle filter using sensor and odometry data when a scan is received. """

        # wait until initialization is complete
        if not(self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated) 
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # calculate the pose of the laser distance sensor 
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
                          frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return

        if self.particle_cloud:

            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold or 
                np.abs(curr_y - old_y) > self.lin_mvmt_threshold or
                np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):

                # This is where the main logic of the particle filter is carried out                
                self.update_particles_with_motion_model()

                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose


    def update_estimated_robot_pose(self):
        """ Based on the particles within the particle cloud, update the robot pose estimate. """

        # compute sum of particle weights for evaluating weighted mean
        weight_sum = sum([particle.w for particle in self.particle_cloud])

        # compute weighted mean x and y coordinates
        x_avg = sum([particle.pose.position.x * particle.w for particle in self.particle_cloud]) / weight_sum
        y_avg = sum([particle.pose.position.y * particle.w for particle in self.particle_cloud]) / weight_sum

        # compute weighted mean of rotation by averaging x- and y- yaw components and recombining them with arctan.
        cos_theta_avg = sum([np.cos(get_yaw_from_pose(particle.pose)) * particle.w for particle in self.particle_cloud]) / weight_sum
        sin_theta_avg = sum([np.sin(get_yaw_from_pose(particle.pose)) * particle.w for particle in self.particle_cloud]) / weight_sum
        theta_avg = math.atan2(sin_theta_avg, cos_theta_avg)

        # create point and quaternion for pose based on weighted averages
        point = Point(x_avg, y_avg, 0.)
        quaternion = Quaternion(*quaternion_from_euler(0., 0., theta_avg, 'rxyz'))

        # updated estimated pose 
        self.robot_estimate = Pose(point, quaternion)
    
    def update_particle_weights_with_measurement_model(self, data):
        """ Computes importance weights for each particle using the likelihood
        field measurement algorithm. """

        # iterate through each particle in the particle cloud to compute its weight
        for particle in self.particle_cloud:

            # initialize particle weight to 1
            weight = 1
            
            # get x, y, and theta positions of particle
            x = particle.pose.position.x
            y = particle.pose.position.y
            theta = get_yaw_from_pose(particle.pose)

            # iterate through sensor measurements for each angle
            for idx, z in enumerate(data.ranges):

                # if an object is detected in a particular direction, find the theoretical location
                #   of the object with respect to the particle, and multiply the weight based on the
                #   distance from this location to the nearest obstacle using a zero-centered Gaussian
                if z != 0.:
                    x_z = x + z * np.cos(theta + math.radians(idx))
                    y_z = y + z * np.sin(theta + math.radians(idx))
                    dist = self.likelihood_field.get_closest_obstacle_distance(x_z, y_z)
                    weight *= compute_prob_zero_centered_gaussian(dist, 0.1)

            # set particle weight
            particle.w = weight


    def update_particles_with_motion_model(self):
        """ Translates and rotates particle positions according to the robot's motion (calculated from
        its odometry). Implementation drawn from sample_motion_model_odometry in Probabilistic Robotics. """

        # grab the current and previous position and rotation of the robot
        curr_x = self.odom_pose.pose.position.x
        old_x = self.odom_pose_last_motion_update.pose.position.x
        curr_y = self.odom_pose.pose.position.y
        old_y = self.odom_pose_last_motion_update.pose.position.y
        curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
        old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

        # compute the robot's translation (delta_trans) and rotation both before and after translation (delta_rot1, delta_rot2)
        delta_rot1 = math.atan2(curr_y - old_y, curr_x - old_x) - curr_yaw
        delta_trans = math.sqrt((curr_x - old_x)**2 + (curr_y - old_y)**2)
        delta_rot2 = curr_yaw - old_yaw - delta_rot1

        # initialize a new particle cloud to be filled with updated particle positionns
        new_particle_cloud = []
        
        # iterate through particles in the particle cloud, updating their positions and adding the updated particles
        #   to new_particle_cloud.
        for particle in self.particle_cloud:

            # grab position and rotation of particle
            x = particle.pose.position.x
            y = particle.pose.position.y
            theta = get_yaw_from_pose(particle.pose)

            # add noise to the robot's movement by sampling a normal distribution before translating and rotating the particle
            delta_rot1_noise = delta_rot1 - sample_normal_distribution(self.alpha_1 * delta_rot1 + self.alpha_2 * delta_trans)
            delta_trans_noise = delta_trans - sample_normal_distribution(self.alpha_3 * delta_trans + self.alpha_4 * (delta_rot1 + delta_rot2))
            delta_rot2_noise = delta_rot2 - sample_normal_distribution(self.alpha_1 * delta_rot2 + self.alpha_2 * delta_trans)

            # compute new position and rotation of the particle by adding in the (noise-supplemented) robot movement
            x_new = x + delta_trans_noise * np.cos(theta + delta_rot1_noise)
            y_new = y + delta_trans_noise * np.sin(theta + delta_rot1_noise)
            theta_new = theta + delta_rot1_noise + delta_rot2_noise

            # intialize new point and quaternion corresponding with the particle pose
            point = Point(x_new, y_new, 0.)
            quaternion = Quaternion(*quaternion_from_euler(0., 0., theta_new, 'rxyz'))
 
            # initialize particle with appropriate pose and weight 1 (weights will be updated in the measurement model step)
            particle_pose = Pose(point, quaternion)
            particle_weight = 1
            particle = Particle(particle_pose, particle_weight)

            # append particle to new_particle_cloud
            new_particle_cloud.append(particle)
        
        # set self.particle_cloud to the new particle cloud
        self.particle_cloud = new_particle_cloud


if __name__=="__main__":
    
    pf = ParticleFilter()

    rospy.spin()


