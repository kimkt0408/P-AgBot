#!/usr/bin/env python

# import
import os
import rospy
import numpy as np
import tf
import math

from sensor_msgs.msg import PointCloud2, PointCloud, ChannelFloat32
import sensor_msgs.point_cloud2 as pc2

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped, Point32
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal
from tf.transformations import quaternion_from_euler


# PARAMETERS (CORN_SIM)
NUM_ROW = 8

POINT_RANGE = 1.2

ROW_WIDTH = 0.7 #[m]
HALF_ROW_WIDTH = 0.5 * ROW_WIDTH
WIDTH_WEIGHT = 0.8
INTERVAL_X = 1.0 / 8.0

MIN_DIST_X = INTERVAL_X * 0.5
MIN_DIST_Y = HALF_ROW_WIDTH * 1
# MIN_DIST = 2

SEARCHING_AREA = INTERVAL_X * 1

OS_TO_BASE_HEIGHT = 0.3195426 # [m] 0.355115
BASE_LINK_HEIGHT = 0.065 #[m]
OS_SENSOR_HEIGHT = BASE_LINK_HEIGHT + OS_TO_BASE_HEIGHT  #[m]

ROOF_HEIGHT = 5
FLOOR_ALTITUDE = -1.0
BOTTOM_THRESHOLD = 0.5

CENTER_X = 0
START_X = 1.0 / 8.0

TOTAL_SIZE_X = 6.0

TOTAL_CROPS_X = int(((TOTAL_SIZE_X - START_X) // INTERVAL_X) + 1)


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


# Crop height monitoring class
class HeightMeasurement:

    def CornPositionCallback(self, corn_position):
        self.corn_positions = corn_position.points


    def PointCloudCallback(self, point_cloud):
        points = pc2.read_points(point_cloud)

        self.points_arr = np.asarray(list(points))

        # Two CEPF corns
        # (self.trans_scan1, self.rot_scan1) = self.listener1.lookupTransform('/map', '/os_sensor', rospy.Time(0))
        (self.trans_scan1, self.rot_scan1) = self.listener1.lookupTransform('/map', '/os1_sensor', rospy.Time(0))


        (self.trans_scan2, self.rot_scan2) = self.listener2.lookupTransform('/odom', '/base_link', rospy.Time(0))
        # (self.trans_scan3, self.rot_scan3) = self.listener3.lookupTransform('/odom', '/base_link', rospy.Time(0))

        self.height_computation()


    def height_computation(self):

        bottom_bool = False

        self.points_x_y_z = list()
        self.points_map = list()
        self.list_points_rows = list() 
        self.list_tallest_points = list()
        self.final_list_tallest_points = list()

        # Point transformation from the frame /os_sensor to /map
        for point in self.points_arr:
            point_x_y_z = point[0:3]
            self.points_x_y_z.append(list(point_x_y_z)) 

        # Set the transformation information: translation & rotation
        q_x = self.rot_scan1[0]
        q_y = self.rot_scan1[1]
        q_z = self.rot_scan1[2]
        q_w = self.rot_scan1[3]

        roll, pitch, yaw = euler_from_quaternion(q_x, q_y, q_z, q_w)

        rot_matrix = np.dot(Rz(yaw), np.dot(Ry(pitch), Rx(roll)))

        floor_altitude_tmp = 0.0

        for point_x_y_z in self.points_x_y_z:

            v1 = np.array([[point_x_y_z[0]], [point_x_y_z[1]], [point_x_y_z[2]]])
            v2 = np.dot(rot_matrix, v1)

            point_map = [v2[0][0], v2[1][0], v2[2][0]]

            point_map[0] += self.trans_scan1[0]
            point_map[1] += self.trans_scan1[1]
            point_map[2] += self.trans_scan1[2]

            self.points_map.append(point_map)

            # Search the floor point in the map
            if ((point_map[2] > FLOOR_ALTITUDE) & (point_map[2] < 0)):
                if (point_map[2] < floor_altitude_tmp):
                    self.floor_point_map = point_map
                    floor_altitude_tmp = point_map[2]

        floor_altitude = self.floor_point_map[2]        

        # Search the point clouds of our interest       
        for i in range(NUM_ROW):
            dist_to_row = HALF_ROW_WIDTH*(NUM_ROW - 2*i - 1) + CENTER_X

            max_interest_dist = dist_to_row + HALF_ROW_WIDTH * WIDTH_WEIGHT
            min_interest_dist = dist_to_row - HALF_ROW_WIDTH * WIDTH_WEIGHT

            list_points_row = list()
            for point_map in self.points_map:

                if ((point_map[1] > min_interest_dist) & (point_map[1] < max_interest_dist)):   # Pick the poincloud in the row
                    if ((point_map[2] > 0) & (abs(point_map[2]) < ROOF_HEIGHT)):                # Pick the pointcloud under the roof
                        list_points_row.append(point_map)

            self.list_points_rows.append(list_points_row)

        # Find the highest point cloud in each row
        for i in range(NUM_ROW):
            height_in_a_row = 0.0
            
            for point_row in self.list_points_rows[i]:
                if (point_row[2] > height_in_a_row):
                    height_in_a_row = point_row[2]
                    the_tallest_point = point_row

            final_height = height_in_a_row + (-floor_altitude)
            self.list_tallest_points.append(the_tallest_point)
            self.final_list_tallest_points.append([the_tallest_point[0], the_tallest_point[1], final_height])

        # Accumulate the highest point cloud in each row
        for i in range(NUM_ROW):
            crop_order = (self.final_list_tallest_points[i][0] - START_X) // INTERVAL_X
            crop_order = int(crop_order)

            if (crop_order >= 0):
                if (self.total_list_tallest_points[i][crop_order][2] < self.final_list_tallest_points[i][2]):
                    self.total_list_tallest_points[i][crop_order] = self.final_list_tallest_points[i]

        # Publish the points in RVIZ
        tallest_points = PointCloud()
        tallest_points.header.frame_id = "map"

        r_ch = ChannelFloat32(name='r')
        g_ch = ChannelFloat32(name='g')
        b_ch = ChannelFloat32(name='b')

        for i in range(len(self.total_list_tallest_points)):
            for j in range(len(self.total_list_tallest_points[i])):

            # tallest_points.points += [Point32(self.corn_xy_height[i][0], self.corn_xy_height[i][1], self.corn_xy_height[i][2])]
                tallest_points.points += [Point32(self.total_list_tallest_points[i][j][0], self.total_list_tallest_points[i][j][1], (self.total_list_tallest_points[i][j][2] + floor_altitude))]

                r_ch.values += [0]
                g_ch.values += [0]
                b_ch.values += [200]

        tallest_points.channels += [r_ch, g_ch, b_ch]
        self.tallest_points_pub.publish(tallest_points)


    # Class Initialization
    def __init__(self):
        self.tallest_points_pub = rospy.Publisher('/tallest_points', PointCloud, queue_size=10)

        self.listener1 = tf.TransformListener()
        self.listener2 = tf.TransformListener()
        self.listener3 = tf.TransformListener()

        self.xy_height_trigger = False

        self.total_list_tallest_points = [[[0.0, 0.0, 0.0] for _ in range(TOTAL_CROPS_X)] for _ in range(NUM_ROW)]
        self.crop_order_tmp = [-1] * NUM_ROW 

        while not rospy.is_shutdown():

            # Simulation: ACRE_CORN, ACRE_SORGHUM
            self.poincloud2_sub = rospy.Subscriber("/os1_cloud_node/points", PointCloud2, self.PointCloudCallback)

            # Two CEPF corns
            # self.poincloud2_sub = rospy.Subscriber("/os_cloud_node/points", PointCloud2, self.PointCloudCallback)

            self.corn_position_sub = rospy.Subscriber("/final_position", PointCloud, self.CornPositionCallback)

            rospy.spin()   


# Main function
if __name__ == "__main__":

    rospy.init_node('Height_measurement')

    while not rospy.is_shutdown():

        try:
            HeightMeasurement().height_computation
            # rospy.spin()

        except KeyboardInterrupt:
            print("Shutting down")

