#!/usr/bin/env python
# license removed for brevity
import rospy
import random
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import time

class Behaviour():
    def __init__(self):
        self.count = 0
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.r = rospy.Rate(10)

    def scan_callback(self,msg):
        regions = {
            'front_l': min(min(msg.ranges[0:44]),10),
            'left': min(min(msg.ranges[45:89]),10),
            'right': min(min(msg.ranges[270:314]),10),
            'front_r': min(min(msg.ranges[315:359]),10),
        }

        self.move(regions)
    

    def odom_callback(self,msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y


    # Obstacle Avoidance
    def move(self, regions):

        msg = Twist()
        d = 0.5

        if self.count==0:
            self.x_init = self.x
            self.y_init = self.y

        if regions['front_l'] > d and regions['front_r'] > d:
            
            self.random_walk(msg)
            
        elif regions['front_l'] < d and regions['front_r'] > d:
            self.count = 0
            print 'front-left obstacle'
            msg.linear.x = 0
            msg.angular.z = -0.3
            self.pub.publish(msg)

        elif regions['front_l'] > d and regions['front_r'] < d:
            self.count = 0  
            print 'front-right obstacle'
            msg.linear.x = 0
            msg.angular.z = 0.3
            self.pub.publish(msg)

        elif regions['front_l'] < d and regions['front_r'] < d:
            self.count = 0
            print 'front obstacle'
            msg.linear.x = 0
            msg.angular.z = 0.3
            self.pub.publish(msg)

        else:
            print 'unknown cases'
            msg.linear.x = 0
            msg.angular.z = 0
            self.pub.publish(msg)


    def random_walk(self, msg):
        if math.sqrt((self.x - self.x_init)**2 + (self.y - self.y_init)**2) < 3:
            self.count=1
            msg.linear.x = 0.3
            msg.angular.z = 0
            print 'random walking'
            self.pub.publish(msg)
        else:
            self.count=0
            msg.linear.x = 0
            msg.angular.z = random.choice([0.3,-0.3])
            print 'random turning'
            self.pub.publish(msg) 
            time.sleep(random.randint(1,5))

def main(): 
    rospy.init_node('behaviours', anonymous=False)
    bot = Behaviour()
    rospy.spin()
 
if __name__ == '__main__':
    main()

