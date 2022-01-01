"""
This file contains all the utility functions to be used.
"""

import cv2
import math
import numpy as np

def get_mask(colour, hsv):
    """Get the binary mask based on the colour wanted"""

    # HSV boundaries for red
    if colour == 'red':
        lower = np.array([0, 200, 20])
        upper = np.array([10, 255, 255])

    # HSV boundaries for blue
    elif colour == 'blue':
        lower = np.array([100, 120, 20])
        upper = np.array([130, 255, 255])

    # HSV boundaries for green
    else:
        lower = np.array([40, 100, 20])
        upper = np.array([70, 255, 255])

    return cv2.inRange(hsv, lower, upper)


def obstacle_avoidance(regions, d, msg):

    """ Using scan callback, detect the front region of the robot to avoid obstacle"""

    # Obstacle avoidance front region of the robot
    if regions['front_l'] > d and regions['front_r'] > d:
        print 'random walking'
        msg.linear.x = 0.3
        msg.angular.z = 0
        
    elif regions['front_l'] < d and regions['front_r'] > d:
        print 'front-left obstacle'
        msg.linear.x = 0
        msg.angular.z = -0.3

    elif regions['front_l'] > d and regions['front_r'] < d:
        print 'front-right obstacle'
        msg.linear.x = 0
        msg.angular.z = 0.3

    elif regions['front_l'] < d and regions['front_r'] < d:
        print 'front obstacle'
        msg.linear.x = 0
        msg.angular.z = 0.2

    else:
        print 'Unknown case'
        msg.linear.x = 0
        msg.angular.z = 0

    return msg


def priority(greenmask, redmask, bluemask):
    """Get the priority of the masks"""

    # Priority mask: green > red > blue
    M_green = cv2.moments(greenmask)
    M_red = cv2.moments(redmask)
    M_blue = cv2.moments(bluemask)

    # If the area of the mask is larger than certain threshold
    if M_green['m00'] > 3500:
        return greenmask, 'green'

    elif M_red['m00'] > 3500:
        return redmask, 'red'

    elif M_blue['m00'] > 3500:
        return bluemask, 'blue'

    else:
        return None, None


def get_centroids(mask):
    """Find centroids of all contours found in the mask"""
    _, cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)

    cX,cY = [], []

    # For each contour found, calculate its centroids, and put in a list
    for c in cnts:
        M = cv2.moments(c)
        if M['m00'] > 3500:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cX.append(cx)
            cY.append(cy)

    return cX, cY


def get_distance(frame, cx, cy):
    """Distance between robot's current position to centroid of the object found"""

    # Distance is taken on certain pixel from the depth image
    distance = float(frame[cy][cx])
    return distance


def draw_centroid(frame, cx, cy):
    """Draw centroid on image"""
    cv2.circle(frame, (cx, cy), 3, (0,0,255), -1)


def get_destination(depth, current_x, current_y, yaw):
    """Calculate the destination of the goal based on the depth"""

    # calculate relative coordinates
    x = depth * math.cos(yaw)
    y = depth * math.sin(yaw)

    # Get absolute coordinates
    x = current_x + x
    y = current_y + y

    return x,y


def check_coordinates(x,y,check):
    """Check if the coordinate is inside a list of explored coordinates"""

    # check is in a form of [(x, y), colour1, (x2,y2), colour2]
    obj = len(check)
    boollist = []

    for i in range(0,obj,3):
        lower = check[i] # lower coordinates
        upper = check[i+1] # upper coordinates

        if lower[0] <= x <= upper[0] and lower[1] <= y <= upper[1]:
            boollist.append(True)
        else:
            boollist.append(False)

    # If check is empty
    if len(check) == 0:
        return False

    else:
        return all(boollist)


def make_range(x,y):
    """set a range of coordinates for the item (make a square of range)"""

    # Get lower left corner and upper right corner of square
    lower = x-0.5, y-0.5
    upper = x+0.5, y+0.5

    return [lower, upper]


def morp_open(mask):
    """Perform morphological operation (erosion then dilation) to remove noise"""

    # Set a default kernel size
    kernel = np.ones((3,3), np.uint8)

    # Erode then dilate
    img_erosion = cv2.erode(mask, kernel, iterations=1)
    img_opened = cv2.dilate(img_erosion, kernel, iterations=1)

    return img_opened


def detect_reached(mask, regions):
    """Detect if the robot has reached the object"""

    # If the white value in the mask cover more than 20% of the mask size and if it is close to an object
    if np.sum(mask == 255) > 0.2*mask.size or regions['front_l'] < 0.3 or regions['front_r'] < 0.3:
        return True
    else:
        return False


def resize_img(img):
    """Resize the image"""
    (h, w) = img.shape[:2]
    image_resized = cv2.resize(img, (w/4,h/4))

    return image_resized


def draw_map(checkcoordinates):
    """Draw the found objects on the map"""

    # Read image
    image = cv2.imread('/home/labuser/catkin_ws/src/COMP4034/src/assignment/maps/train_env.pgm')

    # Flip the image on x axis
    image = cv2.flip(image, 0)
    
    # Length of the checked coordinates
    obj = len(checkcoordinates)

    # In the check list, iterate over all ranges of coordinates and get the colour
    for i in range(0,obj,3):

        px1,py1 = checkcoordinates[i]
        px2,py2 = checkcoordinates[i+1]
        colour = checkcoordinates[i+2]

        # Information of the map from map.yaml
        origx, origy = -10.0, -10.0
        resolution = 0.05
        
        # Get the square bottom left point and upper right point
        start_point = int((px1 - origx)/resolution), int((py1 - origy)/resolution)
        end_point = int((px2 - origx)/resolution), int((py2 - origy)/resolution)

        # Blue color in BGR
        if colour == 'blue':
            color = (255, 0, 0)
        elif colour == 'green':
            color = (0,255,0)
        elif colour == 'red':
            color = (0,0,255)
        else:
            color = (0,0,0)

        # Line thickness of 2 px
        thickness = 1

        # Draw the rectangle out onto the map
        cv2.rectangle(image, start_point, end_point, color, thickness)

    cv2.imshow("Updated Map", image)

