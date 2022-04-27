#!/usr/bin/env python  

#from cmath import inf
from sklearn.cluster import DBSCAN
from threading import local

import os
import roslib
import rospy
import numpy as np
import math
import tf
import geometry_msgs.msg

import random

from sensor_msgs.msg import LaserScan, PointCloud, ChannelFloat32
from geometry_msgs.msg import Pose, Point, Quaternion, Point32
from tf.transformations import quaternion_from_euler

### TWO-CORNS ENVIRONMENT ###

# ANGLE_RESOLUTION = 1.0          # float: 360 / 360
# MIN_SAMPLES1 = 9   #15
# MAX_DISTANCE1 = 0.013 #0.0081              # The maximum distance between two samples for one to be considered as in the neighborhood of the other.

# MIN_SAMPLES2_RATIO = 4    #8
# MIN_SAMPLES2 = 4    #8
# MAX_DISTANCE2 = 0.02              # The maximum distance between two samples for one to be considered as in the neighborhood of the other.

# MIN_RANGE = 0.1                   # The minimum laser scan range to use: To prevent to creating results due to Kinova arm
# MAX_RANGE = 0.65                   # The maximum laser scan range to use: To prevent to creating results due to Kinova arm

# GROUND_TRUTH_RADIUS_SIM = 0.012367

# TIME_PERIOD = 5 #10

# DIST_THRESHOLD = 0.3

# ROW_INTERVAL = 0.7
# DIST_THRESHOLD_X = 0.3          # 0.2
# DIST_THRESHOLD_Y = ROW_INTERVAL * 0.5

# ANGLE_RESOLUTION = 1.0          # float: 360 / 360
# MIN_SAMPLES1 = 8   #15
# MAX_DISTANCE1 = 0.012 #0.0081              # The maximum distance between two samples for one to be considered as in the neighborhood of the other.

# MIN_SAMPLES2_RATIO = 4    #8
# MIN_SAMPLES2 = 4    #8
# MAX_DISTANCE2 = 0.02              # The maximum distance between two samples for one to be considered as in the neighborhood of the other.

# MIN_RANGE = 0.1                   # The minimum laser scan range to use: To prevent to creating results due to Kinova arm
# MAX_RANGE = 1.3                   # The maximum laser scan range to use: To prevent to creating results due to Kinova arm


# GROUND_TRUTH_RADIUS_SIM = 0.012367

# TIME_PERIOD = 5 #10

# DIST_THRESHOLD = 0.3

# ROW_INTERVAL = 0.7
# DIST_THRESHOLD_X = 0.2
# DIST_THRESHOLD_Y = ROW_INTERVAL * 0.5

# ### ACRE-CORN-SIM ENVIRONMENT ###

ANGLE_RESOLUTION = 1.0          # float: 360 / 360
MIN_SAMPLES1 = 8   #15
MAX_DISTANCE1 = 0.018 #0.0081              # The maximum distance between two samples for one to be considered as in the neighborhood of the other.

MIN_SAMPLES2 = 4    #8
MAX_DISTANCE2 = 0.1               # The maximum distance between two samples for one to be considered as in the neighborhood of the other.

MIN_RANGE = 0.1                   # The minimum laser scan range to use: To prevent to creating results due to Kinova arm
MAX_RANGE = 0.65                   # The maximum laser scan range to use: To prevent to creating results due to Kinova arm


GROUND_TRUTH_RADIUS_SIM = 0.012367

TIME_PERIOD = 5 #10

# DIST_THRESHOLD = 0.13
ROW_INTERVAL = 0.7
DIST_THRESHOLD_X = 0.105
DIST_THRESHOLD_Y = ROW_INTERVAL * 0.5

# ANGLE_RESOLUTION = 1.0          # float: 360 / 360
# MIN_SAMPLES1 = 10   #15
# MAX_DISTANCE1 = 0.02 #0.0081              # The maximum distance between two samples for one to be considered as in the neighborhood of the other.

# MIN_SAMPLES2 = 4    #8
# MAX_DISTANCE2 = 0.1               # The maximum distance between two samples for one to be considered as in the neighborhood of the other.

# MIN_RANGE = 0.3                   # The minimum laser scan range to use: To prevent to creating results due to Kinova arm
# MAX_RANGE = 1.5                   # The maximum laser scan range to use: To prevent to creating results due to Kinova arm


# GROUND_TRUTH_RADIUS_SIM = 0.012367

# TIME_PERIOD = 8 #10

# # DIST_THRESHOLD = 0.13
# ROW_INTERVAL = 0.7
# DIST_THRESHOLD_X = 0.105
# DIST_THRESHOLD_Y = ROW_INTERVAL * 0.5

### ACRE-SIM ENVIRONMENT ###

# ANGLE_RESOLUTION = 1.0          # float: 360 / 360
# MIN_SAMPLES1 = 9   #15
# MAX_DISTANCE1 = 0.009 #0.0081              # The maximum distance between two samples for one to be considered as in the neighborhood of the other.

# MIN_SAMPLES2 = 3    #8
# MAX_DISTANCE2 = 0.02             # The maximum distance between two samples for one to be considered as in the neighborhood of the other.

# MIN_RANGE = 0.3                   # The minimum laser scan range to use: To prevent to creating results due to Kinova arm
# MAX_RANGE = 1.2                   # The maximum laser scan range to use: To prevent to creating results due to Kinova arm


# GROUND_TRUTH_RADIUS_SIM = 0.012367

# TIME_PERIOD = 5 #10

# DIST_THRESHOLD = 0.3


def euler_from_quaternion(x, y, z, w):
    # """
    # Convert a quaternion into euler angles (roll, pitch, yaw)
    # roll is rotation around x in radians (counterclockwise)
    # pitch is rotation around y in radians (counterclockwise)
    # yaw is rotation around z in radians (counterclockwise)
    # """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians


def Rx(theta):
  return np.array([[ 1, 0           , 0           ],
                   [ 0, math.cos(theta),-math.sin(theta)],
                   [ 0, math.sin(theta), math.cos(theta)]])
  

def Ry(theta):
  return np.array([[ math.cos(theta), 0, math.sin(theta)],
                   [ 0           , 1, 0           ],
                   [-math.sin(theta), 0, math.cos(theta)]])
  

def Rz(theta):
  return np.array([[math.cos(theta), -math.sin(theta), 0 ],
                   [ math.sin(theta), math.cos(theta) , 0 ],
                   [ 0           , 0            , 1 ]])


def incremental_farthest_search(points, k):
    remaining_points = points[:]
    solution_set = []

    solution_set.append(remaining_points.pop(\
                                             random.randint(0, len(remaining_points) - 1))) 

    for _ in range(k-1):
        distances = [distance(p, solution_set[0]) for p in remaining_points]
        for i, p in enumerate(remaining_points):
            for j, s in enumerate(solution_set):
                distances[i] = min(distances[i], distance(p, s))
        solution_set.append(remaining_points.pop(distances.index(max(distances))))

    return solution_set


def distance(A, B):
    return abs(math.sqrt((A[0]-B[0])**2 + (A[1]-B[1])**2 + (A[2]-B[2])**2))


def circum_radius(point_A, point_B, point_C):
    origin = [0, 0, 0]

    vector_a = [point_C[0] - point_B[0], point_C[1] - point_B[1], point_C[2] - point_B[2]]
    vector_b = [point_A[0] - point_C[0], point_A[1] - point_C[1], point_A[2] - point_C[2]]
    vector_c = [point_B[0] - point_A[0], point_B[1] - point_A[1], point_B[2] - point_A[2]]

    abs_vector_a = distance(vector_a, origin)
    abs_vector_b = distance(vector_b, origin)
    abs_vector_c = distance(vector_c, origin)

    cross_a_b = np.cross(vector_a, vector_b)
    abs_cross_a_b = distance(cross_a_b, origin)

    R = (abs_vector_a*abs_vector_b*abs_vector_c)/(2.0*abs_cross_a_b)

    return R


class DiameterMeasurement():
    def laserCallback(self, laserscan):
        if (self.iter % TIME_PERIOD == 0):
            self.global_laserscan_list = list()

            self.local_laserscan_list_x = [0] * len(laserscan.ranges) * TIME_PERIOD
            self.local_laserscan_list_y = [0] * len(laserscan.ranges) * TIME_PERIOD
            self.local_laserscan_list_z = [0] * len(laserscan.ranges) * TIME_PERIOD
            
            self.global_laserscan_list_x = [0] * len(laserscan.ranges) * TIME_PERIOD
            self.global_laserscan_list_y = [0] * len(laserscan.ranges) * TIME_PERIOD
            self.global_laserscan_list_z = [0] * len(laserscan.ranges) * TIME_PERIOD
        
        (self.trans_scan, self.rot_scan) = self.listener.lookupTransform('/map', '/laser', rospy.Time(0))
        
        q_x = self.rot_scan[0]
        q_y = self.rot_scan[1]
        q_z = self.rot_scan[2]
        q_w = self.rot_scan[3]

        roll, pitch, yaw = euler_from_quaternion(q_x, q_y, q_z, q_w)

        rot_matrix = np.dot(Rz(yaw), np.dot(Ry(pitch), Rx(roll)))
        
        self.local_angle = [0] * len(laserscan.ranges)
        
        for i in range(0, len(laserscan.ranges)):
            self.local_angle[i] = ANGLE_RESOLUTION * i / 180 * math.pi
            self.local_laserscan_list_x[i] = laserscan.ranges[i] * math.cos(self.local_angle[i])
            self.local_laserscan_list_y[i] = laserscan.ranges[i] * math.sin(self.local_angle[i])
            
            # If the range of laserscan is shorter than the minimum threshold of the range 
            if ((laserscan.ranges[i] < MIN_RANGE) | (laserscan.ranges[i] > MAX_RANGE)):
                continue

            v1 = np.array([[self.local_laserscan_list_x[i]], [self.local_laserscan_list_y[i]], [self.local_laserscan_list_z[i]]])            
            v2 = np.dot(rot_matrix, v1)

            self.global_laserscan_list_x[i] = v2[0][0]
            self.global_laserscan_list_y[i] = v2[1][0]
            self.global_laserscan_list_z[i] = v2[2][0]

            self.global_laserscan_list_x[i] = self.global_laserscan_list_x[i] + self.trans_scan[0]
            self.global_laserscan_list_y[i] = self.global_laserscan_list_y[i] + self.trans_scan[1]
            self.global_laserscan_list_z[i] = self.global_laserscan_list_z[i] + self.trans_scan[2]

            self.global_laserscan_list.append([i, self.global_laserscan_list_x[i], self.global_laserscan_list_y[i], self.global_laserscan_list_z[i]])
        
        self.global_stalk_laserscan_list = [[idx, x, y, z] for [idx, x, y, z] in self.global_laserscan_list if abs(y) < 10]
        self.global_stalk_laserscan_list_xyz = [[x, y, z] for [idx, x, y, z] in self.global_laserscan_list if abs(y) < 10]
        self.global_stalk_laserscan_list_xy = [[x, y] for [idx, x, y, z] in self.global_laserscan_list if abs(y) < 10]
        
        self.iter += 1
        if (self.iter % TIME_PERIOD == 0):
            self.Clustering()


    def Clustering(self):
        clustering1 = DBSCAN(eps=MAX_DISTANCE1, min_samples=MIN_SAMPLES1).fit_predict(self.global_stalk_laserscan_list_xy)
        num_clusters1 = max(clustering1) + 1

        # Create a list to classify clusters (x, y, z)
        self.clusters_list = [[] for _ in range(num_clusters1)] 

        for i in range(len(clustering1)):
            if clustering1[i] != -1:
                self.clusters_list[clustering1[i]].append(self.global_stalk_laserscan_list_xyz[i])

        for i in range(num_clusters1):
            self.cluster_points_x = list()
            self.cluster_points_y = list()
            self.cluster_points_z = list()
            mat1 = list()
            mat2 = list()

            # The maximum distance between points
            solution_set_2 = incremental_farthest_search(self.clusters_list[i], 2)

            x_center_2 = 0.5 * (solution_set_2[0][0] + solution_set_2[1][0])
            y_center_2 = 0.5 * (solution_set_2[0][1] + solution_set_2[1][1])
            
            stalk_radius_2 = distance(solution_set_2[0], solution_set_2[1])/2
            
            for j in range(len(self.clusters_list[i])):
                self.cluster_points_x.append(self.clusters_list[i][j][0])
                self.cluster_points_y.append(self.clusters_list[i][j][1])
                self.cluster_points_z.append(self.clusters_list[i][j][2])

                mat1.append([self.clusters_list[i][j][0], self.clusters_list[i][j][1], 1.0])
                mat2.append(-(self.clusters_list[i][j][0]**2 + self.clusters_list[i][j][1]**2))

            X = np.linalg.lstsq(mat1, mat2, rcond=-1)
            
            x_center = -0.5*X[0][0]
            y_center = -0.5*X[0][1]
            radius = math.sqrt((X[0][0]**2 + X[0][1]**2)/4 - X[0][2])
            
            self.x_y_radius_list.append([x_center_2, y_center_2, stalk_radius_2])
            self.x_y_list.append([x_center_2, y_center_2])
        
        # Publish PointCloud in RVIZ for visualization
        self.clustered_pointcloud_list = list()
        self.clustered_pointcloud = PointCloud()
        self.clustered_pointcloud.header.frame_id = "map"

        r_ch = ChannelFloat32(name='r')   
        g_ch = ChannelFloat32(name='g')   
        b_ch = ChannelFloat32(name='b')   

        for i in range(len(clustering1)):
            if clustering1[i] != -1:
                self.clustered_pointcloud.points += [Point32(self.global_stalk_laserscan_list_xyz[i][0], self.global_stalk_laserscan_list_xyz[i][1], self.global_stalk_laserscan_list_xyz[i][2])]
                
                r_ch.values += [150 - clustering1[i] * 20]
                g_ch.values += [clustering1[i] * 20]
                b_ch.values += [100 + clustering1[i] * 20]  
                
                self.clustered_pointcloud.channels += [r_ch, g_ch, b_ch]

        self.clustered_pointcloud_publisher1.publish(self.clustered_pointcloud)

        if self.iter > (TIME_PERIOD * 10):
            self.final_result()


    def get_list(self):
        return self.x_y_radius_list


    def final_result(self):
        self.final_x_y_radius_list = self.x_y_radius_list
        self.final_x_y_list = self.x_y_list
        
        self.clustering2 = DBSCAN(eps=MAX_DISTANCE2, min_samples=MIN_SAMPLES2).fit_predict(self.final_x_y_radius_list)
        
        self.num_clusters2 = max(self.clustering2) + 1

        self.final_clusters_list = [[] for _ in range(self.num_clusters2)] 
        self.final_avg_clusters_list = list()

        for i in range(len(self.clustering2)):
            if self.clustering2[i] != -1:
                self.final_clusters_list[self.clustering2[i]].append(self.final_x_y_radius_list[i])
                
        for i in range(self.num_clusters2):
            self.avg_final_cluster = np.mean(self.final_clusters_list[i], axis=0)
            self.final_avg_clusters_list.append(self.avg_final_cluster.tolist())
                
        delete_list = list()

        for i in range(len(self.final_avg_clusters_list)):
            for j in range(len(self.final_avg_clusters_list)):
                if (j != i):

                    dist_i_j_x = abs(self.final_avg_clusters_list[i][0]-self.final_avg_clusters_list[j][0])
                    dist_i_j_y = abs(self.final_avg_clusters_list[i][1]-self.final_avg_clusters_list[j][1])
                    dist_i_j = math.sqrt(dist_i_j_x**2 + dist_i_j_y**2 )

                    if ((dist_i_j_x < DIST_THRESHOLD_X) & (dist_i_j_y < DIST_THRESHOLD_Y)):

                        count_i = np.sum(1 for x in self.clustering2 if x == i)
                        count_j = np.sum(1 for x in self.clustering2 if x == j)
                        if (count_i > count_j):
                            delete_list.append(j)
                        elif (count_i < count_j):
                            delete_list.append(i)
        
        self.final_avg_clusters_list2 = np.delete(self.final_avg_clusters_list, delete_list, axis=0)
        self.final_avg_clusters_list = self.final_avg_clusters_list2.tolist()

        for i in range(len(self.final_avg_clusters_list)):
            print("RADIUS: ", self.final_avg_clusters_list[i])

        final_pointcloud = PointCloud()
        final_pointcloud.header.frame_id = "map"

        r_ch = ChannelFloat32(name='r')   
        g_ch = ChannelFloat32(name='g')   
        b_ch = ChannelFloat32(name='b')   
        
        for i in range(len(self.final_avg_clusters_list)):
            if abs(self.final_avg_clusters_list[i][1]) > 0.2:
                final_pointcloud.points += [Point32(self.final_avg_clusters_list[i][0], self.final_avg_clusters_list[i][1], self.final_avg_clusters_list[i][2])]

                r_ch.values += [0]
                g_ch.values += [255]
                b_ch.values += [0]  

                final_pointcloud.channels += [r_ch, g_ch, b_ch]

        self.clustered_pointcloud_publisher2.publish(final_pointcloud)


    # Class Initialization
    def __init__(self):

        self.listener = tf.TransformListener()   

        self.clustered_pointcloud_publisher1 = rospy.Publisher('/clustered_pointcloud', PointCloud, queue_size=10)
        self.clustered_pointcloud_publisher2 = rospy.Publisher('/final_position', PointCloud, queue_size=10)
    
        self.x_y_radius_list = list()
        self.x_y_list = list()

        self.iter = 0 

        while not rospy.is_shutdown():
            self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.laserCallback)
            
            rospy.spin()     


if __name__ == '__main__':

    rospy.init_node('diameter_measurement')
    while not rospy.is_shutdown():
        try:
            DiameterMeasurement()

        except KeyboardInterrupt:
            print("Shutting down")

    print("Shutting down")
