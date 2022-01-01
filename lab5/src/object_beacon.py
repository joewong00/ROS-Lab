#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from utils import obstacle_avoidance

class ObjectBeacon:
    def __init__(self):

        # Number of objects
        self.green = 0
        self.red = 0
        self.blue = 0
        
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def move(self, cx, img, regions):
        """P control movement"""

        (_, w) = img.shape[:2]
        err = cx - w/2
        msg = Twist()

        # If the front region of the laser scan if larger than 0.25m
        if regions['front_l'] > 0.25 and regions['front_r'] > 0.25:
            print "Moving" 
            msg.linear.x = 0.2
            msg.angular.z = -float(err) / 500 
            self.pub.publish(msg)
          
        # Else, perform obstacle avoidance
        else:
            msg = obstacle_avoidance(regions, 0.25, msg)
            self.pub.publish(msg)
         

    def reached(self,colour):
        """Print reach message"""

        if colour == 'green':
            self.green += 1
            print "Green box #"+str(self.green)+" is found!"
                
        elif colour == 'red':
            self.red += 1
            print "Red fire hydrant #"+str(self.red)+" is found!"

        elif colour == 'blue':
            self.blue += 1
            print "Blue mail box #"+str(self.blue)+" is found!"

        # Then perform a 7s turn
        self.turn(0.25, 7)


    def stop(self):
        """Stop the robot from moving"""
        msg = Twist()
        msg.linear.x = 0.
        msg.angular.z = 0
        self.pub.publish(msg)


    def turn(self, angular_v, sec):
        """Turning the robot based on certain time frame and angular velocity"""
        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = angular_v

        t = rospy.Time.now().to_sec()

        while rospy.Time.now().to_sec() - t < rospy.Duration(sec).to_sec() and not rospy.is_shutdown():
            self.pub.publish(msg)