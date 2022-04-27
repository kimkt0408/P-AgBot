#!/usr/bin/env python
import os
import rospy
import numpy as np
import tf
import math
import time

import actionlib

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
#from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
#from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

# Parameters

# # ACRE-Sorghum
# intensity_threshold = 100                                # Change
# # forward_radius_threshold = 1.2                                # Change
# forward_radius_threshold = 1.6                                # Change

# backward_radius_threshold = 2.2
# # line_extraction_period = 10    # per 4s = 20/(300rpm/60)

# angle_resolution = 1    # 360 / 360
# # dx = 1
# velocity = 0.03     # 0.15
# dx = velocity
# theta_weight = 0.06 #0.75  #0.6 #0.5
# near_area_dist = 0.4
# next_goal_dist_threshold = 0.2#0.21
# angular_velocity_in_place = 0.8
# direction_threshold = -0.8

# micro_control_weight = 0.5

# forward_angle_start = 40
# forward_angle_range = 45   # angle(degree) = angle_range * angle_resolution

# backward_angle_start = 100  #100
# backward_angle_range = 60   #70 # angle(degree) = angle_range * angle_resolution



# ACRE-Sorghum
# intensity_threshold = 100                                # Change
# # forward_radius_threshold = 1.2                                # Change
# forward_radius_threshold = 1.6#1.6                                # Change

# backward_radius_threshold = 2.2
# # line_extraction_period = 10    # per !4s = 20/(300rpm/60)

# angle_resolution = 1    # 360 / 360
# dx = 1       # 0.08
# velocity = 0.1     # 0.15
# theta_weight = 1 #0.75  #0.6 #0.5
# near_area_dist = 0.5
# next_goal_dist_threshold = 0.22
# angular_velocity_in_place = 1.0
# direction_threshold = -0.8

# micro_control_weight = 0.6

# forward_angle_start = 30
# forward_angle_range = 90   # angle(degree) = angle_range * angle_resolution

# backward_angle_start = 80  #100
# backward_angle_range = 20  #30 #70 # angle(degree) = angle_range * angle_resolution

# ACRE-Corn
intensity_threshold = 100                                # Change
# forward_radius_threshold = 1.2                                # Change
forward_radius_threshold = 1.6                                # Change

backward_radius_threshold = 2.2
# line_extraction_period = 10    # per 4s = 20/(300rpm/60)

angle_resolution = 1    # 360 / 360
# dx = 1
velocity = 0.1     # 0.15
dx = velocity
theta_weight = 0.05 #0.75  #0.6 #0.5
near_area_dist = 0.4
next_goal_dist_threshold = 0.2#0.21
angular_velocity_in_place = 0.8
direction_threshold = -0.8

micro_control_weight = 0.5

forward_angle_start = 10
forward_angle_range = 75   # angle(degree) = angle_range * angle_resolution

backward_angle_start = 100  #100
backward_angle_range = 70   #70 # angle(degree) = angle_range * angle_resolution


# Define arrays
pose_robot = np.array([0, 0])
cur_dir_robot = np.array([1, 0])

# Autonomous navigation class
class NaviIntegration:
    
    # AMCL pose callback function
    def PoseListener(self):

        # localization pose based on AMCL
        self.amcl_pose_x = self.trans_pose[0]
        self.amcl_pose_y = self.trans_pose[1]
        

        # np.savetxt(b_file, [self.amcl_pose_x ], fmt='%f')
        # np.savetxt(b_file, [self.amcl_pose_y ], fmt='%f')

        # localization pose direction based on AMCL
        amcl_orientation_x = self.rot_pose[0]
        amcl_orientation_y = self.rot_pose[1]
        amcl_orientation_z = self.rot_pose[2]
        amcl_orientation_w = self.rot_pose[3]

        # Change from Quarternion to Euler angle (x, y, z, w -> roll, pitch, yaw)
        amcl_orientation_list = [amcl_orientation_x, amcl_orientation_y, amcl_orientation_z, amcl_orientation_w]
        (self.amcl_roll, self.amcl_pitch, self.amcl_yaw) = tf.transformations.euler_from_quaternion(amcl_orientation_list)


    # Goal info callback function
    def GoalCallback(self, goal):

        # Goal position
        self.goal_pose_x = goal.goal.target_pose.pose.position.x
        self.goal_pose_y = goal.goal.target_pose.pose.position.y
        
        # Goal orientation (Quarternion)
        goal_orientation_x = goal.goal.target_pose.pose.orientation.x
        goal_orientation_y = goal.goal.target_pose.pose.orientation.y
        goal_orientation_z = goal.goal.target_pose.pose.orientation.z
        goal_orientation_w = goal.goal.target_pose.pose.orientation.w

        # Change from Quarternion to Euler angle (x, y, z, w -> roll, pitch, yaw)
        goal_orientation_list = [goal_orientation_x, goal_orientation_y, goal_orientation_z, goal_orientation_w]
        (self.goal_roll, self.goal_pitch, self.goal_yaw) = tf.transformations.euler_from_quaternion(goal_orientation_list)
  

    # Laser scan callback function
    def laserCallback(self, laserscan):
        (self.trans_pose,self.rot_pose) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            
        self.PoseListener()
        
        # Compute the distance between the robot and the surroundings        
        self.direction_function()

        if self.dot_dir > 0:    # When the robot drives forward
            self.left_laser_ranges = laserscan.ranges[forward_angle_start:forward_angle_start+forward_angle_range]   # 0 ~ 360
            self.left_laser_intensities = laserscan.intensities[forward_angle_start:forward_angle_start+forward_angle_range]

            self.right_laser_ranges = laserscan.ranges[-(forward_angle_start+forward_angle_range):-forward_angle_start]
            self.right_laser_intensities = laserscan.intensities[-(forward_angle_start+forward_angle_range):-forward_angle_start]

            # print(laserscan.ranges[backward_angle_start:backward_angle_start+backward_angle_range])
        else:    # When the robot drives backward
            self.left_laser_ranges = laserscan.ranges[backward_angle_start:backward_angle_start+backward_angle_range]   # 0 ~ 360
            self.left_laser_intensities = laserscan.intensities[backward_angle_start:backward_angle_start+backward_angle_range]

            self.right_laser_ranges = laserscan.ranges[-(backward_angle_start+backward_angle_range):-backward_angle_start]
            self.right_laser_intensities = laserscan.intensities[-(backward_angle_start+backward_angle_range):-backward_angle_start]
        
        self.dist_computation()

        self.move()


    # Class Initialization
    def __init__(self):

        self.listener = tf.TransformListener()
    
        points_seq = rospy.get_param('move_base_seq/p_seq')
        # Only yaw angle required (no rotations around x and y axes) in deg:
        yaweulerangles_seq = rospy.get_param('move_base_seq/yea_seq')
        
        #List of goal quaternions:
        quat_seq = list()

        #List of goal poses:
        self.pose_seq = list()
        self.goal_cnt = 0

        for yawangle in yaweulerangles_seq:
            #Unpacking the quaternion list and passing it as arguments to Quaternion message constructor
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
        
        n = 3

        # Returns a list of lists [[point1], [point2],...[pointn]]
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
        for point in points:
            #Exploit n variable to cycle in quat_seq
            self.pose_seq.append(Pose(Point(*point),quat_seq[n-3]))
            n += 1
                
        #Create action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(0.0))
        
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return

        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
            
        while not rospy.is_shutdown():

            self.movebase_client()      # Set the goal point          
            
            self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.laserCallback)
        
            rospy.spin() 


    # Distance computation function
    def dist_computation(self):
    
        list_left_laser = []
        list_right_laser = []

        # 1. Choose the laser scans in the area of interest
        # (1) Left side
        for left_point_range in self.left_laser_ranges:
            
            if self.dot_dir > 0:    # When the robot drives forward
                radius_threshold = forward_radius_threshold
            else:                   # When the robot drives backward
                radius_threshold = backward_radius_threshold

            if left_point_range < radius_threshold:                 # (i) Near the robot
                
                left_point_idx = self.left_laser_ranges.index(left_point_range)
                left_point_intensity = self.left_laser_intensities[left_point_idx]
                
                list_left_laser.append([left_point_idx, left_point_range, left_point_intensity])

        # (2) Right side
        for right_point_range in self.right_laser_ranges:

            if right_point_range < radius_threshold:                # (i) Near the robot
                right_point_idx = self.right_laser_ranges.index(right_point_range)
                right_point_intensity = self.right_laser_intensities[right_point_idx]
                
                list_right_laser.append([right_point_idx, right_point_range, right_point_intensity])
    

        # 2. Compute the distance between the robot and the chosen laser scans
        # (1) Left side
        left_trigger = False
        right_trigger = False

        for left_point in list_left_laser:
            angle = angle_resolution * left_point[0]    
            angle = math.radians(angle)
            dist = abs(left_point[1] * math.sin(angle))
            
            if (dist != 0.0):              
                if left_trigger == False:
                    min_left_dist = dist
                    left_trigger = True
                else:
                    if min_left_dist > dist:
                        min_left_dist = dist
        
        # (2) Right side
        for right_point in list_right_laser:
            angle = angle_resolution * right_point[0]
            angle = math.radians(angle)
            dist = abs(right_point[1] * math.sin(angle))

            if (dist != 0.0):
                if right_trigger == False:
                    min_right_dist = dist
                    right_trigger = True
                else:
                    if min_right_dist > dist:
                        min_right_dist = dist

        # 3. Determine the direction of driving
        # Compute the average value of left/right minimum distance
        half_dist = 0.5 * (min_left_dist + min_right_dist)
        diff_dist = min_left_dist - half_dist

        # Compute the direction and angular velocity
        self.theta = math.atan(diff_dist/dx)

        self.rot_mat = np.array([[math.cos(self.theta), -math.sin(self.theta)],[math.sin(self.theta), math.cos(self.theta)]])
        self.goal_dir_robot = np.matmul(self.rot_mat, cur_dir_robot)


    # The function to command a linear/angular velocity
    def direction_function(self):                
        self.goal.target_pose.pose = self.pose_seq[self.goal_cnt]

        # Compare between directions of AMCL pose and goal
        dir_amcl = np.array([math.cos(self.amcl_yaw), math.sin(self.amcl_yaw)])
        dir_robot_goal = np.array([self.goal.target_pose.pose.position.x - self.amcl_pose_x, self.goal.target_pose.pose.position.y - self.amcl_pose_y])

        self.dot_dir = np.dot(dir_amcl, dir_robot_goal) / (np.linalg.norm(dir_amcl) * np.linalg.norm(dir_robot_goal))
        self.cross_dir = np.cross(dir_amcl, dir_robot_goal)

        # Compute the distance from the robot to the goal point
        self.dist_amcl_goal = math.sqrt(math.pow((self.goal.target_pose.pose.position.x - self.amcl_pose_x), 2) + math.pow((self.goal.target_pose.pose.position.y - self.amcl_pose_y), 2))
        
        self.trigger = np.ones(len(self.pose_seq), dtype=bool)

        print("Distance:", self.dist_amcl_goal)
        print("Direction:", self.dot_dir)


    def move(self):   
        # /cmd_vel publisher
        cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        vel_msg = Twist()

        if self.dot_dir < 0:                 # When the robot needs a rotation in place to reach the goal position             
            if self.dist_amcl_goal > near_area_dist:     # When the robot is near the goal point: "Fine" controller   
                beginTime = rospy.Time.now()
                secondsIWantToSendMessagesFor = rospy.Duration(1); 
                endTime = secondsIWantToSendMessagesFor + beginTime

                while((rospy.Time.now() < endTime) & self.rotation_trigger1 == True):
                    vel_msg.linear.x = 0
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0
                    
                    vel_msg.angular.x = 0
                    vel_msg.angular.y = 0

                    if self.cross_dir >= 0:
                        vel_msg.angular.z = - (math.pi - np.arccos(self.dot_dir))
                    else:
                        vel_msg.angular.z = math.pi - np.arccos(self.dot_dir)

                    cmd_vel_publisher.publish(vel_msg)

                if self.rotation_trigger1 == True:
                    rospy.sleep(1)

                self.rotation_trigger1 = False

                vel_msg.linear.x = - velocity
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = - self.theta * theta_weight

                print("Backward row tracing...:", vel_msg.angular.z)

                print(self.dist_amcl_goal)
                print(near_area_dist)

            else:
                beginTime = rospy.Time.now()
                secondsIWantToSendMessagesFor = rospy.Duration(1); 
                endTime = secondsIWantToSendMessagesFor + beginTime

                while((rospy.Time.now() < endTime) & self.rotation_trigger2 == True):
                    vel_msg.linear.x = 0
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0
                    
                    vel_msg.angular.x = 0
                    vel_msg.angular.y = 0

                    if self.cross_dir >= 0:
                        vel_msg.angular.z = - (math.pi - np.arccos(self.dot_dir))
                    else:
                        vel_msg.angular.z = math.pi - np.arccos(self.dot_dir)

                    cmd_vel_publisher.publish(vel_msg)

                    print("Rotating in place, angular velocity: ", vel_msg.angular.z)
                    
                if self.rotation_trigger2 == True:
                    rospy.sleep(1)

                self.rotation_trigger2 = False

                vel_msg.linear.x = - velocity * micro_control_weight
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = 0

                print("Go backward slowly") 

        else:

            if self.dist_amcl_goal > near_area_dist:     # When the robot is near the goal point: "Fine" controller   
                
                beginTime = rospy.Time.now()
                secondsIWantToSendMessagesFor = rospy.Duration(1); 
                endTime = secondsIWantToSendMessagesFor + beginTime

                while((rospy.Time.now() < endTime) & self.rotation_trigger1 == True):
                    
                    vel_msg.linear.x = 0
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0
                    
                    vel_msg.angular.x = 0
                    vel_msg.angular.y = 0

                    if self.cross_dir >= 0:
                        vel_msg.angular.z = np.arccos(self.dot_dir)
                    else:
                        vel_msg.angular.z = -np.arccos(self.dot_dir)

                    cmd_vel_publisher.publish(vel_msg)

                    print("Rotating in place, angular velocity: ", vel_msg.angular.z)

                if self.rotation_trigger1 == True:
                    rospy.sleep(1)

                self.rotation_trigger1 = False

                vel_msg.linear.x = velocity
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = self.theta * theta_weight

                print("Forward row tracing...:", vel_msg.angular.z)
            
            else:
                beginTime = rospy.Time.now()
                secondsIWantToSendMessagesFor = rospy.Duration(1); 
                endTime = secondsIWantToSendMessagesFor + beginTime

                while((rospy.Time.now() < endTime) & self.rotation_trigger2 == True):
                    
                    vel_msg.linear.x = 0
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0
                    
                    vel_msg.angular.x = 0
                    vel_msg.angular.y = 0

                    if self.cross_dir >= 0:
                        vel_msg.angular.z = np.arccos(self.dot_dir)
                    else:
                        vel_msg.angular.z = -np.arccos(self.dot_dir)

                    cmd_vel_publisher.publish(vel_msg)

                    print("Rotating in place, angular velocity: ", vel_msg.angular.z)

                if self.rotation_trigger2 == True:
                    rospy.sleep(1)

                self.rotation_trigger2 = False

                vel_msg.linear.x = velocity * micro_control_weight
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = 0

                print("Go forward slowly") 

        cmd_vel_publisher.publish(vel_msg)

        # Change to the next goal point
        if (self.dist_amcl_goal < next_goal_dist_threshold) & self.trigger[self.goal_cnt]:     # When the robot is far from the goal point: "row tracer" controller                                    
        
            self.trigger[self.goal_cnt] = False
            self.goal_cnt += 1
            
            if self.goal_cnt >= len(self.pose_seq):
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                return
            else:
                print("!!!!!!Change the goal!!!!!!!!")
                self.goal.target_pose.pose = self.pose_seq[self.goal_cnt] 
                self.rotation_trigger1 = True   
                self.rotation_trigger2 = True

        cmd_vel_publisher.publish(vel_msg)
        

    def movebase_client(self):
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now() 

        self.rotation_trigger1 = True
        self.rotation_trigger2 = True


# Main function
if __name__ == "__main__":

    rospy.init_node('navigation_goal')

    while not rospy.is_shutdown():
        try:
            NaviIntegration()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue