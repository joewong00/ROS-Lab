# Finding Objects

![](../gif/findobjects.gif)

### Robot's Behaviour
- Waypoints navigation
  - Several waypoints from the map are set for robot to navigate around the map
  - Navigation is done using navigation stack
- Object beaconing
  - Green boxes, red fire hydrant, blue mail boxes
  - Proportional control towards the centroid of the object
- Object reaching
  - When white pixel in the mask contain more than 30% of the image, then turn around and continue navigation