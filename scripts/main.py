#!/usr/bin/env python

import rospy
import math
import tf
import numpy as np
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Quaternion
from std_msgs.msg import Bool


LINEAR_VEL = 0.22
STOP_DISTANCE = 0.4
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
count = []


class Operation():
    def __init__(self):
        rospy.init_node('factory', anonymous=False)
        #rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.r = rospy.Rate(10)

        #-------------------------Add Station--------------------------------

        rospy.Subscriber('station1', Bool, self.callback1)
        rospy.Subscriber('station2', Bool, self.callback2)
        rospy.Subscriber('station3', Bool, self.callback3)

        #--------------------------------------------------------------------
        self.program()
        rospy.spin() #IMPORTANT-keep python excuted and existing until this node is stop

    def callback1(self,data):
        if data.data == True:
            rospy.logwarn('Station1 Running...')
            count.append(1)
            print(count)
        else:
            rospy.logfatal('Station1 Stop !!!')
            count.remove(1)

    def callback2(self,data):
        if data.data == True:
            rospy.logwarn('Station2 Running...')
            count.append(2)
            print(count)
        else:
            rospy.logfatal('Station2 Stop !!!')
            count.remove(2)
        
    def callback3(self,data):
        if data.data == True:
            rospy.logwarn('Station3 Running...')
            count.append(3)
            print(count)
        else:
            rospy.logfatal('Station3 Stop !!!')
            count.remove(3)

    def get_station(self):
        if count[0] == 1:
            pos = [0.2,0]
            ang = 0 
        elif count[0] == 2:
            pos = [2,0]
            ang = 0
        elif count[0] == 3:
            pos = [3,0]
            ang = 0
        return (pos[0], pos[1], ang) #-180 < ang <180, position(mm)

    def get_scan(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []
       
        samples = len(scan.ranges)  # The number of samples is defined in 
                                    # turtlebot3_<model>.gazebo.xacro file,
                                    # the default is 360.
        samples_view = 1            # 1 <= samples_view <= samples
        
        if samples_view > samples:
            samples_view = samples

        if samples_view is 1:
            scan_filter.append(scan.ranges[0])

        else:
            left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
            right_lidar_samples_ranges = samples_view//2
            
            left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
            scan_filter.extend(left_lidar_samples + right_lidar_samples)

        for i in range(samples_view):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = 3.5
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 0
        
        return scan_filter

    def get_point(self, goal_x, goal_y, goal_z, position, rotation):
        self.odom_frame = 'odom'
        position = Point()
        move_cmd = Twist()
        self.tf_listener = tf.TransformListener()

        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")

        #(position, rotation) = self.get_odom()
        last_rotation = 0
        linear_speed  = 0.3
        angular_speed = 0.3
        goal_z = np.deg2rad(goal_z)
        goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        distance = goal_distance

        while distance > 0.05:
            #(position, rotation) = self.get_odom()
            x_start = position.x
            y_start = position.y
            path_angle = atan2(goal_y - y_start, goal_x- x_start)

            if path_angle < -pi/4 or path_angle > pi/4:
                if goal_y < 0 and y_start < goal_y:
                    path_angle = -2*pi + path_angle
                elif goal_y >= 0 and y_start > goal_y:
                    path_angle = 2*pi + path_angle
            if last_rotation > pi-0.1 and rotation <= 0:
                rotation = 2*pi + rotation
            elif last_rotation < -pi+0.1 and rotation > 0:
                rotation = -2*pi + rotation
            move_cmd.angular.z = angular_speed * path_angle-rotation

            distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
            move_cmd.linear.x = min(linear_speed * distance, 0.1)

            if move_cmd.angular.z > 0:
                move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
            else:
                move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

            last_rotation = rotation
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()
        (position, rotation) = self.get_odom()

        while abs(rotation - goal_z) > 0.05:
            (position, rotation) = self.get_odom()
            if goal_z >= 0:
                if rotation <= goal_z and rotation >= goal_z - pi:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.1
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.1
            else:
                if rotation <= goal_z + pi and rotation > goal_z:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.1
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.1
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(move_cmd())
    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])

    def program(self):
        turtlebot_moving = True
        move_cmd = Twist()
        while not rospy.is_shutdown():
            if len(count) > 0:
                (x,y,z) = self.get_station()
                (pos, rot) = self.get_odom()
                lidar_distances = self.get_scan()
                min_distance = min(lidar_distances)
                if min_distance < SAFE_STOP_DISTANCE:
                    if turtlebot_moving:
                        move_cmd.linear.x = 0.0
                        move_cmd.linear.y = 0.0
                        move_cmd.angular.z = 0.0
                        self.cmd_vel.publish(move_cmd)
                        turtlebot_moving = False
                        rospy.loginfo('Stop!')
                else:
                    turtlebot_moving = True
                    rospy.loginfo('Distance of the obstacle : %f', min_distance)
                    self.get_point(x, y, z, pos, rot)
            else:
                pass
            
    
    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)



if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            Operation()
    except: 
        rospy.loginfo("Shutdown Program")
        