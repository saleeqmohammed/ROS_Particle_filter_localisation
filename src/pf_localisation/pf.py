from geometry_msgs.msg import Pose, PoseArray, Quaternion
from  pf_base import PFLocaliserBase
import math
import rospy

from . util import rotateQuaternion, getHeading
from random import random, vonmisesvariate,gauss,randrange

from time import time

class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # ----- Set motion model parameters
 
        # ----- Sensor model parameters
        self.INITIALPOSE = (0,0,0)
        self.PARTICLECOUNT =100
        self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict
        self.particlecloud = self.initialise_particle_cloud(0)
       
    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise

        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.
        
        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """
        print(initialpose)
        startingPoses =PoseArray
        #create random uniform probability positoins
        initialPositions =[(randrange(-15,15),randrange(-15,15)) for x in range(self.PARTICLECOUNT)]
        #create random probability angles
        initialAngles = [math.cos(vonmisesvariate(0,0)/2)*0.5  for x in range(self.PARTICLECOUNT)]
        for i in range(self.PARTICLECOUNT):
            currPose = Pose
            currPose.position.x=initialPositions[i][0]
            currPose.position.y = initialPositions[i][1]
            currPose.orientation.x=0
            currPose.orientation.y=0
            currPose.orientation.z=1
            currPose.orientation.w=initialAngles[i]
            startingPoses.poses.append(currPose)
        return PoseArray

 
    
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
        pass
    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).
        
        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.
        
        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after 
        throwing away any which are outliers

        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
         """
        pass
    