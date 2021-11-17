#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import time
import numpy as np

class TurtlebotDriving():
    def __init__(self):
        # initiliaze
        rospy.init_node('turtlebotdriving', anonymous=False)  
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=10)
        self.pose_x = []  # record x trajectory
        self.pose_y = []  # record y trajectory
        self.r = rospy.Rate(10)
        self.drawsquare()


    def odom_callback(self, msg):
       
        # get x and y positions and store in a list
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.pose_x.append(self.x)
        self.pose_y.append(self.y)


    def drawsquare(self):

        iter = input("OPENLOOP: Enter square-motion iteration:")

        # variables for moving the robot
        linear_v = 0.25
        angular_v = 0.3
        forward_time = 4.5
        turn_time = 5.3
        finalpose_x = np.array([])
        finalpose_y = np.array([])

        for i in range(iter):
            print"drawing a square .. #",i+1
            time.sleep(3)

            self.forward(linear_v, forward_time)
            self.turn(angular_v, turn_time)
            self.forward(linear_v, forward_time)
            self.turn(angular_v, turn_time)
            self.forward(linear_v, forward_time)
            self.turn(angular_v, turn_time)
            self.forward(linear_v, forward_time)
            self.turn(angular_v, turn_time)
            self.stop()

            print"final location for iter#",i+1
            print"x = ",self.x
            print"y = ",self.y
            finalpose_x = np.append(finalpose_x, self.x)
            finalpose_y = np.append(finalpose_y, self.y)

        print"Covariance Matrix =",self.motion_error_modelling(finalpose_x, finalpose_y)

    # Move robot forward
    def forward(self, v, num_secs):

        # initial time
        t = rospy.Time.now().to_sec()
        msg = Twist()
        msg.linear.x = v
        print("Going Straight")

        while rospy.Time.now().to_sec() - t < rospy.Duration(num_secs).to_sec() and not rospy.is_shutdown():
            self.pub.publish(msg)
            self.r.sleep()

    # Turn the robot
    def turn(self, v, num_secs):

        # initial time
        t = rospy.Time.now().to_sec()
        msg = Twist()
        msg.angular.z = v
        print("Turning") 

        while rospy.Time.now().to_sec() - t < rospy.Duration(num_secs).to_sec() and not rospy.is_shutdown():
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


