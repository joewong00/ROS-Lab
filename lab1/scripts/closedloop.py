#!/usr/bin/env python
# license removed for brevity
import rospy
import tf
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import math
import time
import numpy as np

class TurtlebotDriving():
    def __init__(self):
        # initiliaze
        rospy.init_node('turtlebotdriving', anonymous=False)  
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10, latch=True)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=10)
        self.pose_x = []  # record x trajectory
        self.pose_y = []  # record y trajectory
        self.pose = Pose2D()
        self.r = rospy.Rate(10)
        self.drawsquare()


    def odom_callback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        self.pose_x.append(self.pose.x)
        self.pose_y.append(self.pose.y)


    def drawsquare(self):

        iter = input("CLOSEDLOOP : Enter square-motion iteration:")

        linear_v = 0.25
        angular_v = 0.25
        finalpose_x = np.array([])
        finalpose_y = np.array([])

        for i in range(iter):
            print"drawing a square .. #",i+1
            time.sleep(3)
            desired_theta = 0

            # initial orientation
            self.init_orient(0.01)

            desired_theta += math.pi/2
            self.forward(linear_v, (1,0))
            self.turn(0.1,0)          # make sure the angle is aligned before first turn
            self.turn(angular_v, desired_theta)

            desired_theta += math.pi/2
            self.forward(linear_v, (1,1))
            self.turn(angular_v, desired_theta)

            desired_theta += math.pi/2
            self.forward(linear_v, (0,1))
            self.turn(angular_v, desired_theta)

            desired_theta += math.pi/2
            self.forward(linear_v, (0,0))
            self.turn(angular_v, desired_theta)

            self.stop()
            print"final location for iter",i+1
            print"x = ",self.pose.x
            print"y = ",self.pose.y
            finalpose_x = np.append(finalpose_x, self.pose.x)
            finalpose_y = np.append(finalpose_y, self.pose.y)
        
        print"Covariance Matrix =",self.motion_error_modelling(finalpose_x, finalpose_y)

    # Ensure robot in correct orientation
    def init_orient(self, v):
        print("Orienting")
        if self.pose.theta < 0:
            msg = Twist()
            msg.angular.z = v
            while self.pose.theta < 0 and not rospy.is_shutdown():
                self.pub.publish(msg)
                self.r.sleep() 
        else:
            msg = Twist()
            msg.angular.z = -v
            # setting an offset of 0.5 so that it does not overturn and become negative
            while self.pose.theta > 0.5 and not rospy.is_shutdown():
                self.pub.publish(msg)
                self.r.sleep() 


    # Move the robot forward
    def forward(self, v, desired):

        # desired position
        desired_x, desired_y = desired

        x_init = self.pose.x
        y_init = self.pose.y

        d = math.sqrt((desired_x - x_init)**2 + (desired_y- y_init)**2)

        msg = Twist()
        msg.linear.x = v
        print("Going Straight")

        while math.sqrt((self.pose.x - x_init)**2 + (self.pose.y - y_init)**2) < d and not rospy.is_shutdown():
            self.pub.publish(msg)
            self.r.sleep()


    # Turn the robot
    def turn(self, v, angle):

        msg = Twist()
        msg.angular.z = v
        print("Turning")

        # if desired angle is 0 or 2pi
        if angle == 2*math.pi or angle == 0:  
            while self.pose.theta < 0 and not rospy.is_shutdown():
                self.pub.publish(msg)
                self.r.sleep()
        else:
            while (self.pose.theta % (2*math.pi)) < angle and not rospy.is_shutdown():
                self.pub.publish(msg)
                self.r.sleep()


    # Stop the robot
    def stop(self):
        print("Stop")
        stop_msg = Twist() 
        self.pub.publish(stop_msg)

    # Plot the trajectory of the robot path
    def plot_trajectory(self):
        plt.figure()
        plt.title('Robot Path')
        plt.plot(self.pose_x, self.pose_y)
        plt.show()
        
    # Calculate the covariance matrix
    def motion_error_modelling(self,finalx,finaly):
        
        x_bar = np.mean(finalx)
        y_bar = np.mean(finaly)
        P = [[0,0],[0,0]] #covariance matrix

        P[0][0] = np.mean((finalx - x_bar)**2)
        P[0][1] = np.mean((finalx - x_bar)*(finaly - y_bar))
        P[1][0] = np.mean((finaly - y_bar)*(finalx - x_bar))
        P[1][1] = np.mean((finaly - y_bar)**2)

        return P

        
def main(): 
    rospy.init_node('turtlebotdriving', anonymous=False)  
    bot = TurtlebotDriving()
    bot.plot_trajectory()
 
if __name__ == '__main__':
    main()


