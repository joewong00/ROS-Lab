# Use Computer Vision to Move Towards Green Objects

## Image processing in OpenCV

![](/gif/follower.gif)

- Using Image callback to obtain image captured by the robot, cv2_bridge is used to convert the image into cv2 BRG format
- Obtain the mask of a green objects captured in the image
- Calculate the centroid of the mask (green object), and implement a P-control to move towards the centroid, avoiding obstacles while moving
- If there are more than one green objects, only focus on leftmost green object
- If no green object is detected, robot performs random walk and avoids obstacles