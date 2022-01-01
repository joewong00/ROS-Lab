#!/usr/bin/env python
import numpy as np
import tf
import rospy
from sensor_msgs.msg import Image, LaserScan
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose2D, Point
from nav_msgs.msg import Odometry
import cv2, cv_bridge
import message_filters
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from object_beacon import ObjectBeacon

from utils import get_mask, get_centroids, priority, get_destination, get_distance, check_coordinates, make_range, draw_centroid, detect_reached, morp_open, resize_img, draw_map


class Train(object):

    def __init__(self):

        # Initialize 
        self.check = []
        self.bridge = cv_bridge.CvBridge()
        self.pose = Pose2D()
        self.beaconing = ObjectBeacon()
        self.waypoint = [
            (-1.2, 4, -1.57),
            (-1,-2, 1.57),
            (0.5,-0.5, 0),
            (0.5, -4, 0),
            (3.5,-4, 1.57),
            (3,-0.5, 1.57),
            (0,3, 1.57),
            (2,2, 0),
            (4,4, 3.14),
            (5,1, 0),
            (6,4, -1.57),
            (6,-4, 1.57)
        ]
        
        # Subscribe to odometry to get robot's odometry information
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Subscribe to laser scan to get robot's lidar sensor information
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Subscribe to both depth image and rgb image from the robot
        self.imagergb_sub = message_filters.Subscriber('camera/rgb/image_raw', Image)
        self.imagedepth_sub = message_filters.Subscriber('camera/depth/image_raw', Image)


    def scan_callback(self, msg):
        """Get a range of sensors at the front of the robot for obstacle avoidance"""

        # Left, front-left, front-right and right region of the robot
        self.regions = {
            'front_l': min(min(msg.ranges[0:44]),10),
            'left': min(min(msg.ranges[45:89]),10),
            'right': min(min(msg.ranges[270:314]),10),
            'front_r': min(min(msg.ranges[315:359]),10),
        }


    def odom_callback(self, msg):
        """Get (x, y, theta) specification from odometry topic"""

        # Get quarternion of the robot
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

        # x, y, theta of the robot
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y


    def image_callback(self, imagergb, imagedepth):
        """Process image by using cv2 for the main function"""

        # Convert ROS image message to OpenCV image 
        self.depth_frame = self.bridge.imgmsg_to_cv2(imagedepth, desired_encoding="32FC1")
        self.rgb_frame = self.bridge.imgmsg_to_cv2(imagergb, "bgr8")

        # Resize the depth and rgb images
        self.depth_frame = resize_img(self.depth_frame)
        self.rgb_frame = resize_img(self.rgb_frame)

        # Convert the depth frame to bgr image and convert rgb image to hsv
        self.depth_frame = cv2.GaussianBlur(self.depth_frame, (5,5) , 0)
        hsv = cv2.cvtColor(self.rgb_frame, cv2.COLOR_BGR2HSV)
        self.bgr_depth = cv2.cvtColor(self.depth_frame, cv2.COLOR_GRAY2BGR)
        
        # Get the respective mask based on the hsv, then perform morphological opening
        self.greenmask = morp_open(get_mask('green', hsv))
        self.redmask = morp_open(get_mask('red', hsv))
        self.bluemask = morp_open(get_mask('blue', hsv))

        # Combine all 3 masks
        self.mask = self.greenmask + self.redmask + self.bluemask
        
        # Get the centroids of the blobs found in the mask
        self.cX, self.cY = get_centroids(self.mask)

        # If there are centroids found
        if self.cX:

            # Get the leftmost centroid
            self.cx = min(self.cX)
            self.cy = self.cY[np.argmin(self.cX)]

            # Calculate the distance of the centroid using the depth
            self.depth = get_distance(self.depth_frame, self.cx, self.cy)

            # Draw the centroid in the images
            draw_centroid(self.depth_frame, self.cx, self.cy)
            draw_centroid(self.mask, self.cx, self.cy)
            draw_centroid(self.rgb_frame, self.cx, self.cy)

        # Display images (rgb, mask and depth image)
        cv2.imshow("Camera", self.rgb_frame)
        cv2.imshow("Depth", self.depth_frame)
        cv2.imshow("Mask", self.mask)
        cv2.waitKey(3)
    

    def navigate(self):
        """Main navigation function"""

        # loop through all waypoints
        for i, point in enumerate(self.waypoint):

            # Centroid not detected
            if not self.cX:
        
                # Navigate
                print "moving to waypoint ",i+1
                j = 0
                reachedgoal = False
        
                # While the goal is not reached and no centroid detected and given 3 chances
                while not reachedgoal and not self.cX and j<3:

                    # Navigate to the waypoint
                    reachedgoal = self.moveToGoal(point[0], point[1], point[2])
                    j += 1

            # Centroid detected
            else:
            
                reachedgoal = False
                j=0
                print "centroid detected", i+1
                
                # While there are centroids
                while len(self.cX) > 0:
                
                    # Based on red, green, blue mask, get the highest priority mask and colour
                    priormask, colour = priority(self.greenmask, self.redmask, self.bluemask)

                    # If depth of the object can be calculated
                    if not np.isnan(self.depth):
                        
                        # Based on the depth of the object, calculate the world coordinate
                        goalx, goaly = get_destination(self.depth, self.pose.x, self.pose.y, self.pose.theta)

                        # Check if we have been to the place of the object before
                        if check_coordinates(goalx, goaly, self.check):
                            print 'We have been there before!'
                            self.beaconing.turn(0.25, 5)

                        # Else, P control towards the object
                        else:
                            print 'Moving to object'
                            self.beaconing.move(self.cx, priormask, self.regions)

                            # If considered as reached object
                            if detect_reached(self.mask, self.regions):
                                self.beaconing.reached(colour)

                                # Record the coordinates and make range of it for drawing square, put them inside a checklist
                                coordinatesrange = make_range(goalx, goaly)
                                self.check += coordinatesrange
                                self.check += [colour]

                # While the goal is not reached and no centroid detected and given 3 chances
                while not reachedgoal and not self.cX and j<3:
                    reachedgoal = self.moveToGoal(point[0], point[1], point[2])
                    j += 1

        # Finally, draw the resulted map
        draw_map(self.check)

                           
    def moveToGoal(self, xGoal, yGoal, yaw):
        """Move base navigation function"""

        # Subscribe to simple action client
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        #wait for the action server to come up
        while(not self.ac.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for the move_base action server to come up")

        # Make MoveBaseGoal object
        goal = MoveBaseGoal()

        # Information of the goal
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Position of the goal
        goal.target_pose.pose.position = Point(xGoal, yGoal, 0)
        
        # Orientation of robot when reaching the goal
        orientation = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = orientation[0]
        goal.target_pose.pose.orientation.y = orientation[1]
        goal.target_pose.pose.orientation.z = orientation[2]
        goal.target_pose.pose.orientation.w = orientation[3]


        rospy.loginfo("Sending goal location ...")

        # Send the goal, wait for the result for 10 seconds
        self.ac.send_goal(goal)
        self.ac.wait_for_result(rospy.Duration(10))
        
        # If the result is not succeeded, return false
        if self.ac.get_state() != GoalStatus.SUCCEEDED:
            return False

        else:
            return True

def main(): 

    # Create Train object
    train = Train()

    # Initalize node
    rospy.init_node('findobjects')

    # Use message filters for synchronous subscriber
    ts = message_filters.ApproximateTimeSynchronizer([train.imagergb_sub, train.imagedepth_sub], 10, 0.1)
    ts.registerCallback(train.image_callback)

    # Wait for 10 seconds before navigating
    rospy.sleep(10)
    train.navigate()

    rospy.spin()
 
if __name__ == '__main__':
    main()