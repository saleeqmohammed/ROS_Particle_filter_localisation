from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_localisation.pf_base import PFLocaliserBase
import math
import rospy
from .util import rotateQuaternion, getHeading
from random import  vonmisesvariate,gauss,uniform
import numpy as np  
from time import time
from statistics import mean,stdev
from . import dbscan
class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        # ----- Set motion model parameters
        self.ODOM_TRANSLATION_NOISE =0.015
        self.ODOM_DRIFT_NOISE=0.030
        self.ODOM_ROTATION_NOISE=0.022
        # ----- Sensor model parameters
        self.PARTICLECOUNT =200
        self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict
        self.ENTROPY_LIMIT =12
        self.PARIICLE_RETENTION = 0.99
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
        initialAngles = [vonmisesvariate(0,0) for x in range(self.PARTICLECOUNT)]
        for _ in range(self.PARTICLECOUNT):
            currPose = Pose()
            currPose.position.x = initialPositions_x[_]
            currPose.position.y = initialPositions_y[_]
            currPose.orientation=rotateQuaternion(Quaternion(w=1.0),initialAngles[_])
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
        #get weights for each particle using self.sensor_model.get_weigth(scan,particle)
        particles =self.particlecloud.poses

        probabilistic_weights=[self.sensor_model.get_weight(scan,particle) for particle in particles]
        #normalize weights (sum to 1)
        #probabilistic_weights =[probability/sum(probabilistic_weights) for probability in probabilistic_weights]
        weighted_particles =[]
        for idx in range(len(particles)):
            weighted_particles.append((particles[idx],probabilistic_weights[idx]))
        #[(particle,weight) for particle,weight in zip(particles,probabilistic_weights)]
        #sort for highest weight/probability
        sorted_particles = sorted(weighted_particles,key= lambda particle_with_weight: particle_with_weight[1],reverse=True)
        heaviestParticles =sorted_particles[0:int(self.PARIICLE_RETENTION*self.PARTICLECOUNT)]
        #for AMCL, calculate entropy / varience for entropy:
        #calculate h(x)=-sum(p(x)*log(p(x)))
        total_entropy =-np.dot(probabilistic_weights,np.log(probabilistic_weights))
        rospy.loginfo("Entropy: %d"%(total_entropy))
        '''
        #FIrst sampling
        N_sample_space =0
        sample_space=[]
        while N_sample_space <= self.PARTICLECOUNT:
            particle_with_weight =sorted_particles.pop()
            sample_space.append(particle_with_weight)
            N_sample_space = N_sample_space + 1
            #ospy.loginfo("stuck here!")
        '''
        #add random particles
        doped_particles=[]
        N_random_particles = self.PARTICLECOUNT -len(heaviestParticles)
        particles_per_gaussian = int(N_random_particles/7)+1
        doping_origins=[(-11,5),(-3,12),(-5,7),(0,0),(11,-5),(3,-12),(5,-7)]
        for origin in doping_origins:
            for _ in range(particles_per_gaussian):
                randPose= Pose()
                doping_x =gauss(origin[0],3)
                doping_y =gauss(origin[1],3)
            #create random uniform probability angles
                doping_angle = math.cos(vonmisesvariate(0,0)/2)
                randPose.position.x = doping_x
                randPose.position.y = doping_y
                randPose.orientation= rotateQuaternion(Quaternion(w=1),doping_angle)
                doped_particles.append(randPose)
        remainingparticles = self.PARTICLECOUNT-len(heaviestParticles)
        addedParticles =0
        dopping_stash=PoseArray()
        while addedParticles < remainingparticles:
            dopping_stash.poses.append(doped_particles[addedParticles])
            addedParticles = addedParticles+1

        total_weight = sum(probabilistic_weights)
        #resampling with cumulative weight etc lecture 6
        cumulative_distribution=[]
        cumulative_weight = 0
        for (particle,weight) in heaviestParticles:
            #print(weight)
            #print(total_weight)
            cumulative_distribution.append((particle,cumulative_weight+ (weight/total_weight)))
            cumulative_weight = cumulative_weight+(weight/total_weight)
        
        resampled_space=PoseArray()
        cycleNum =0
        #print(particle_cumulative_weight[0])
        #generate a random thresholds
        #draw  desired number of points accorrding to systematic resampling technique
        M=len(heaviestParticles)
        threshold =uniform(0,(1/M))

        for points in range(0,M):
            while threshold>cumulative_distribution[cycleNum][1]:
                cycleNum += 1
                
            #construct the particle
            newParticle = Pose()
            sample = cumulative_distribution[cycleNum][0]
            newParticle.position.x = sample.position.x + gauss(0,self.ODOM_TRANSLATION_NOISE)
            newParticle.position.y = sample.position.y + gauss(0, self.ODOM_DRIFT_NOISE)
            newParticle.orientation = rotateQuaternion(Quaternion(w=1),getHeading(sample.orientation) + vonmisesvariate(0,self.ODOM_ROTATION_NOISE))
            resampled_space.poses.append(newParticle)
            threshold =threshold + math.pow(M,-1)
        #return particle cloud
        self.particlecloud.poses = resampled_space.poses + dopping_stash.poses
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
        rospy.loginfo("estimate function called!")
        #A variety of methods can be adopted here we are implementing positional clustering
        #defining a pose object
        estimatedPose = Pose()
        #maximum distance between particles in cluster
        epsilon =3
        #mininum number of particles in a cluster 
        min_particles =2
        #perform a density based spatial clustering based on position
        prominent_cluster =dbscan.prominent_cluster(epsilon,min_particles,self.particlecloud.poses)
        cluster_x =0
        cluster_y =0
        cluster_w =0
        cluster_ori_z =0
        n_cluster =0
        for point in prominent_cluster:
            cluster_x = cluster_x + point.position.x
            cluster_y = cluster_y + point.position.y
            cluster_w = cluster_w + point.orientation.w
            cluster_ori_z = cluster_ori_z + point.orientation.z
            n_cluster = n_cluster+1
        cluster_x = cluster_x/n_cluster
        cluster_y = cluster_y/n_cluster
        cluster_w = cluster_w/n_cluster
        cluster_ori_z = cluster_ori_z/n_cluster
        estimatedPose.position.x = cluster_x
        estimatedPose.position.y = cluster_y
        estimatedPose.orientation.w = cluster_w
        estimatedPose.orientation.z = cluster_ori_z
        return estimatedPose

        

        

