from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_localisation.pf_base import PFLocaliserBase
import math
import rospy
from . util import rotateQuaternion, getHeading
from random import random, vonmisesvariate,gauss,randrange
import numpy as np  
from time import time
from statistics import mean,stdev
class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        # ----- Set motion model parameters
 
        # ----- Sensor model parameters
        self.PARTICLECOUNT =100
        self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict
        #self.particlecloud = self.initialise_particle_cloud(0)
        #--visualisation parameters setting these manually atm since /map_metadata is not subscribed yet
        self.MAP_RESOULUTION = 0.050
        self.MAP_HEIGHT =602*0.050
        self.MAP_WIDTH =602*0.050

    def initialise_particle_cloud(self, initialpose:Pose):
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
        startingPoses = PoseArray()
        #create x and y coordinates of possible posses in a normal distribution around inital position
        initialPositions_x =[gauss(initialpose.pose.pose.position.x,self.MAP_WIDTH/8) for _ in range(self.PARTICLECOUNT)]
        initialPositions_y =[gauss(initialpose.pose.pose.position.y,self.MAP_HEIGHT/8) for _ in range(self.PARTICLECOUNT)]
        #create random uniform probability angles
        initialAngles = [math.cos(vonmisesvariate(0,0)/2)  for x in range(self.PARTICLECOUNT)]
        for _ in range(self.PARTICLECOUNT):
            currPose = Pose()
            currPose.position.x = initialPositions_x[_]
            currPose.position.y = initialPositions_y[_]
            currPose.orientation.x=0
            currPose.orientation.y=0
            currPose.orientation.z=1
            currPose.orientation.w=initialAngles[_]
            startingPoses.poses.append(currPose)
            
        return startingPoses

 
    
    def update_particle_cloud(self, scan):
        #correction
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
        pass
    def estimate_pose(self):
        #Prediction
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
        #A variety of methods can be adopted here we are implementing positional clustering
        #defining a pose object
        estimatedPose = Pose()

        #calculating distance to origin for all particles in the particle cloud
        distances =[self.distanceToParticle(particle) for particle in self.particlecloud]

        

