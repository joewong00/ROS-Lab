#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import cv2, cv_bridge

class Follower:
   def __init__(self, regions):
      self.regions = regions
      self.bridge = cv_bridge.CvBridge()
      self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
      self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
      self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

   # Scanner callback to determine obstacle distance
   def scan_callback(self, msg):
      self.regions = {
         'front_l': min(min(msg.ranges[0:44]),10),
         'left': min(min(msg.ranges[45:89]),10),
         'right': min(min(msg.ranges[270:314]),10),
         'front_r': min(min(msg.ranges[315:359]),10),
        }

   # Image callback to get image captured
   def image_callback(self, msg):
      # Image preprocessing
      image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
      (h, w) = image.shape[:2]
      image_resized = cv2.resize(image, (w/4,h/4))
      hsv = cv2.cvtColor(image_resized, cv2.COLOR_BGR2HSV)

      # Extract green colour and put in mask
      mask = self.get_green_mask(hsv)

      # Calculate the moment of the mask
      moment = cv2.moments(mask)

      # If there is green object
      if moment['m00'] > 0:
         self.green_beaconing(mask, image_resized)

      # No green object
      else:
         print "No green object found!"
         self.obstacle_avoidance(self.regions, mask)

      cv2.imshow("Mask", mask)
      cv2.imshow("Centroid", image_resized)
      cv2.waitKey(3)

   def get_green_mask(self, hsv):
      # Green boundary
      lower_green = np.array([40,  50, 10])
      upper_green = np.array([70, 255, 190])
      return cv2.inRange(hsv, lower_green, upper_green)

   def green_beaconing(self, mask, img):

      (h, w) = img.shape[:2]
      # Find contours of all green objects
      _, cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)

      # Initialize
      i = 0
      cx = [None]*len(cnts)
      cy = [None]*len(cnts)

      # For each contour found
      for c in cnts:
         M = cv2.moments(c)
         if M['m00'] > 0:
            cx[i] = int(M["m10"] / M["m00"])
            cy[i] = int(M["m01"] / M["m00"])
            i = i+1
      
      # Remove any None value in list
      cx = filter(None, cx)
      cy = filter(None, cy)

      # Take the left centroid (min(cx) and corresponding cy)
      cv2.circle(img, (min(cx), cy[np.argmin(cx)]), 3, (0,0,255), -1)
      err = min(cx) - w/2
      
      # Beacon towards and avoid obstacles
      if self.regions['front_l'] > 0.4 and self.regions['front_r'] > 0.4:
         print "Green object found!"
         twist = Twist()
         twist.linear.x = 0.2
         twist.angular.z = -float(err) / 500 
         self.pub.publish(twist)

      else:
         self.obstacle_avoidance(self.regions, mask) 

   # Obstacle avoidance algorithm
   def obstacle_avoidance(self, regions, mask):
      msg = Twist()
      d = 0.4
      green = False
      # If mask is more than 40% of the image, considered object reached
      if np.sum(mask == 255) > 0.4*mask.size:
         green = True

      if regions['front_l'] > d and regions['front_r'] > d and not green:
         msg.linear.x = 0.2
         msg.angular.z = 0
         self.pub.publish(msg)

      elif regions['front_l'] < d and regions['front_r'] > d and not green:
         print 'front-left obstacle'
         msg.linear.x = 0
         msg.angular.z = -0.2
         self.pub.publish(msg)

      elif regions['front_l'] > d and regions['front_r'] < d and not green:
         print 'front-right obstacle'
         msg.linear.x = 0
         msg.angular.z = 0.2
         self.pub.publish(msg)

      elif regions['front_l'] < d and regions['front_r'] < d and not green:
         print 'front obstacle'
         msg.linear.x = 0
         msg.angular.z = 0.2
         self.pub.publish(msg)

      else:
         print 'Green object reached!'
         msg.linear.x = 0 
         msg.angular.z = 0
         self.pub.publish(msg)
         rospy.spin()


rospy.init_node('follower')
regions = {
   'front_l': 0,
   'left': 0,
   'right': 0,
   'front_r': 0
}
follower = Follower(regions)
rospy.spin()
