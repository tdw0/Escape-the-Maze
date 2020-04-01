#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, cv2, cv_bridge
import numpy as np
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class MazeSolver:
        
        def __init__(self):
                # initialise node and set the rate
                rospy.init_node('MazeSolver')
                self.rate = rospy.Rate(10)
                
                # velocity/laser/odom/image sensors
                self.vel = Twist()
                self.laser = None
                self.odom = None

                # laserscan and odometer subscribers
                rospy.Subscriber('/scan', LaserScan, self.laserscan_callback)
                #rospy.Subscriber('/odom', Odometry, self.odom_callback)

                # publisher for velocity
                self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size = 5)
                
                # image stuff (wip)
                self.bridge = cv_bridge.CvBridge()

                self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
                self.image_pub = rospy.Publisher("image_topic_2",Image)
                
                # for moving the robot
                self.mv = Twist()
                self.mv.linear.x = 0.0
                self.mv.angular.z = 0.0

                # initialise variables for wall detection
                self.f_range = [] # front
                self.r_range = [] # right
                self.l_range = [] # left
                
                self.f_min = 0 # front
                self.r_min = 0 # right
                self.l_min = 0 # left

                self.near_wall  = False
                self.red_tile   = False
                self.green_tile = False
                rospy.sleep(1)
                self.solve_maze()
                
        def solve_maze(self):
                while not rospy.is_shutdown():
                        if (self.green_tile == True):
                                print("Goal reached, congratulations.")
                                break
                        while ((self.near_wall == False and not rospy.is_shutdown()) or self.red_tile == True):
                                if (self.red_tile == True):
                                        self.opposite_wall()
                                        #print("stopped")
                                        break
                                
                                # move to wall
                                self.move_to_wall()
                                print(self.near_wall)
                        else:
                                self.follow_wall()

        def laserscan_callback(self, laser_msg):
                self.laser = laser_msg
                # value at in front of robot (90 degrees)
                self.f_range = laser_msg.ranges[211:428]
                #value to the right of robot (180 degrees)
                self.l_range = laser_msg.ranges[429:639]
                # value to the left of robot (0 degrees)
                self.r_range = laser_msg.ranges[0:210]
                
                self.f_min = min(self.f_range)
                self.r_min = min(self.r_range)
                self.l_min = min(self.l_range)

                #print("f_min: ", self.f_min)
                #print("r_min: ", self.r_min)
                #print("l_min: ", self.l_min)

        # if red is detected call solve_maze again
        def image_callback(self, msg):
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                h, w = cv_image.shape[:2]
                hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
                
                # lower red range
                lower_red = np.array([0,120,70])
                upper_red = np.array([10,255,255])
                r_l_mask = cv2.inRange(hsv, lower_red, upper_red)
                # upper red range
                lower_red = np.array([170,120,70])
                upper_red = np.array([180,255,255])
                r_u_mask = cv2.inRange(hsv,lower_red,upper_red)
                # combine masks
                r_mask = r_l_mask + r_u_mask

                # lower green range
                l_u_green = np.array([25, 52, 72])
                # upper green range
                h_u_green = np.array([102, 255, 255])
                # mask
                g_mask = cv2.inRange(hsv, l_u_green, h_u_green)

                # crop to only see 1 line of pixels
                # this will only show the red tile when it is directly infront of the robot
                cropped_r_mask = r_mask[ 419:420, 0:w ]
                cropped_g_mask = g_mask[ 419:420, 0:w ]

                cv2.imshow("img_window", cv_image)
                cv2.waitKey(3)

                # if the mask has a red pixel, avoid the tile
                if cv2.countNonZero(cropped_r_mask) > 0:
                        print("red tile detected")
                        self.red_tile = True
                
                #if the robot is on a green tile, stop
                if cv2.countNonZero(cropped_g_mask) == w:
                        self.green_tile = True

                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

        def move_to_wall(self):
                #print('moving towards a wall')
                self.mv.angular.z = -0.2
                self.cmd_vel_pub.publish(self.mv)
                if (self.f_min > 0.5 and self.r_min > 0.5 and self.l_min > 0.5): # checking if anything is near
                        #print(self.f_range)
                        #print(self.r_min)
                        self.mv.angular.z = -0.1
                        self.mv.linear.x = 0.15 # go forward
                elif (self.r_min < 0.5):
                        #print(self.l_min)
                        self.near_wall = True
                        #print('near left wall')
                else:
                        self.mv.angular.z = -0.25
                        self.mv.linear.x = 0.0

                self.cmd_vel_pub.publish(self.mv)
                self.red_tile = False
        
        def follow_wall(self):
                if (self.f_min > 0.5): # start following the left wall
                        if (self.r_min < 0.35 or self.l_min < 0.35 or self.f_min < 0.35 or math.isnan(self.r_min)): # if too close to the left wall reverse a bit
                                print(self.r_min, ': Too close, Reversing')
                                self.mv.angular.z = -0.8
                                self.mv.linear.x = -0.4
                        elif(self.r_min > 0.5): # follow by zig-zagging
                                print(self.r_min)
                                print('following wall')
                                self.mv.angular.z = -1.2
                                self.mv.linear.x = 0.15
                        else:
                                print('Turning right')
                                self.mv.angular.z = 1.2
                                self.mv.linear.x = 0.15
                else:
                        self.turn_corner()
                
                self.cmd_vel_pub.publish(self.mv)

        def turn_corner(self):
                print('obstacle detected, turning')
                self.mv.angular.z = 1.0
                self.mv.linear.x = 0.0
                self.cmd_vel_pub.publish(self.mv)
                while(self.r_min < 0.3 and not rospy.is_shutdown()):
                        pass
        
        def detect_red_tile(self):
                pass

        # moves the robot to the opposite wall (if there's a red square or the robot is stuck in a loop)
        def opposite_wall(self):
                #rotate 90 degrees
                print("red tile detected, turning")
                self.mv.angular.z = 1.0
                self.cmd_vel_pub.publish(self.mv)
                rospy.sleep(2)
                # move to a new wall
                self.red_tile = False
                self.near_wall = False
                self.solve_maze()

        def shutdown(self):
                # stop turtlebot
                rospy.loginfo("Stop TurtleBot")
                # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
                self.cmd_vel.publish(Twist())
                # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
                rospy.sleep(1)

if __name__ == "__main__":
        maze_solver = MazeSolver()
        rospy.spin()