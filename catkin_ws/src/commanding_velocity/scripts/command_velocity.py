#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
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

                self.near_wall = False
                rospy.sleep(1)
                self.solve_maze()

                
                
        def solve_maze(self):
                while not rospy.is_shutdown():
                        while (self.near_wall == False and not rospy.is_shutdown()):
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


        def move_to_wall(self):
                #print('moving towards a wall')
                if (self.f_min > 0.5 and self.r_min > 0.5 and self.l_min > 0.5): # checking if anything is near
                        #print(self.f_range)
                        #print(self.l_min)
                        self.mv.angular.z = -0.1
                        self.mv.linear.x = 0.15 # go forward
                elif (self.l_min < 0.5):
                        #print(self.l_min)
                        self.near_wall = True
                        #print('near left wall')
                else:
                        self.mv.angular.z = -0.25
                        self.mv.linear.x = 0.0

                self.cmd_vel_pub.publish(self.mv)
        
        def follow_wall(self):
                if (self.f_min > 0.5): # start following the left wall
                        if (self.l_min < 0.35 or self.r_min < 0.35 or self.f_min < 0.35): # if too close to the left wall reverse a bit
                                print(self.l_min, ': Too close, Reversing')
                                self.mv.angular.z = -1.2
                                self.mv.linear.x = -0.3
                        elif(self.l_min > 0.5): # follow by zig-zagging
                                print(self.l_min)
                                print('following wall')
                                self.mv.angular.z = 1.2
                                self.mv.linear.x = 0.15
                        else:
                                print('Turning right')
                                self.mv.angular.z = -1.2
                                self.mv.linear.x = 0.15
                else:
                        self.turn_corner()
                
                self.cmd_vel_pub.publish(self.mv)

        def turn_corner(self):
                print('obstacle detected, turning')
                self.mv.angular.z = -1.0
                self.mv.linear.x = 0.0
                self.cmd_vel_pub.publish(self.mv)
                while(self.l_min < 0.3 and not rospy.is_shutdown()):
                        pass


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