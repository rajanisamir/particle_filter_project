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

from random import randint, random

def sample_normal_distribution(b):
    """ A helper function that samples from a normal distirbution with variance b.
        Implementation taken from Probabilistic Robotics. """
    return (b / 6) * sum([-1 + 2 * np.random.random() for i in range(12)])

def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw


def draw_random_sample(sample_list, n, probabilities):
    """ Draws a random sample of n elements from a given list of choices and their specified probabilities.
    We recommend that you fill in this function using random_sample.
    """
    return np.random.choice(sample_list, size=n, replace=True, p=probabilities)


class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w



class ParticleFilter:

    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False        


        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map
        self.map = OccupancyGrid()

        # the number of particles used in the particle filter
        self.num_particles = 1000

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we perform an update
        self.lin_mvmt_threshold = 0.2        
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None


        # Setup publishers and subscribers

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

        self.map = data
    

    def initialize_particle_cloud(self):
        
        # Wait until get_map callback sets the map by checking if frame_id
        #  has been populated.
        while self.map.header.frame_id == '':
            continue

        # Store list of all open spaces in map in available_indices. Choose
        #  particle positions uniformly at random, with replacement; store
        #  these positions in chosen_indices.
        available_indices = []
        for i in range(self.map.info.width * self.map.info.height):
            if self.map.data[i] == 0:
                available_indices.append(i)
        chosen_indices = draw_random_sample(available_indices, self.num_particles, None)
        
        # For each chosen particle index, initialize a particle with the appropriate
        #  coordinates and a random yaw angle in the interval [0, 360). Assign the
        #  weights as 1; they will be normalized in normalize_particles.
        for idx in chosen_indices:
            idx_y = int(idx / self.map.info.width)
            idx_x = idx % self.map.info.width
            y = idx_y * self.map.info.resolution + self.map.info.origin.position.y
            x = idx_x * self.map.info.resolution + self.map.info.origin.position.x
            theta = np.random.random() * 360

            point = Point(x, y, 0.)
            quaternion = Quaternion(*quaternion_from_euler(0., 0., theta, 'rxyz'))

            particle_pose = Pose(point, quaternion)
            particle_weight = 1
            particle = Particle(particle_pose, particle_weight)
            self.particle_cloud.append(particle)
        
        # Normalize particle weights and publish particle cloud.
        self.normalize_particles()

        self.publish_particle_cloud()


    def normalize_particles(self):
        
        # make all the particle weights sum to 1.0
        weight_sum = sum(particle.w for particle in self.particle_cloud)
        for particle in self.particle_cloud:
            particle.w /= weight_sum
        
    def publish_particle_cloud(self):

        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        # particle_cloud_pose_array.poses

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)


    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)


    def resample_particles(self):

        # TODO

        pass


    def robot_scan_received(self, data):

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
        # based on the particles within the particle cloud, update the robot pose estimate
        
        # TODO

        pass

    
    def update_particle_weights_with_measurement_model(self, data):

        # TODO

        pass


    def update_particles_with_motion_model(self):
        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly
        curr_x = self.odom_pose.pose.position.x
        old_x = self.odom_pose_last_motion_update.pose.position.x
        curr_y = self.odom_pose.pose.position.y
        old_y = self.odom_pose_last_motion_update.pose.position.y
        curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
        old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

        delta_rot1 = math.atan2(curr_y - old_y, curr_x - old_x) - curr_yaw
        delta_trans = math.sqrt((curr_x - old_x)**2 + (curr_y - old_y)**2)
        delta_rot2 = curr_yaw - old_yaw - delta_rot1

        new_particle_cloud = []
        
        for particle in self.particle_cloud:
            x = particle.pose.position.x
            y = particle.pose.position.y
            theta = get_yaw_from_pose(particle.pose)

            # Here, we should incorporate noise: see algorithm in Probabilistic Robotics book.

            x_new = x + delta_trans * np.cos(theta + delta_rot1)
            y_new = y + delta_trans * np.sin(theta + delta_rot1)
            theta_new = theta + delta_rot1 + delta_rot2

            point = Point(x_new, y_new, 0.)
            quaternion = Quaternion(*quaternion_from_euler(0., 0., theta_new, 'rxyz'))

            particle_pose = Pose(point, quaternion)
            particle_weight = 1
            particle = Particle(particle_pose, particle_weight)
            new_particle_cloud.append(particle)
        
        self.particle_cloud = new_particle_cloud



if __name__=="__main__":
    
    pf = ParticleFilter()

    rospy.spin()


