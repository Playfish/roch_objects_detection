# roch_objects_detection
-----------------------------

Roch objects detection is a simple demo for check out ```circle``` and ```rectangular``` object and publish modifid images thought topics named ```/detection/objects```

# Installation

For install roch_objects detection code following command:
```
cd catkin_ws/src # change directory into your catkin workspace.
git clone https://github.com/Playfish/roch_objects_detection.git
cd ..
catkin_make
```
Once finish you can use this demo.

# Usage

Due to based on Roch, so you should following command:

```
roslaunch roch_bringup minimal.launch # Run Roch divers
roslaunch roch_bringup sensor.launch # Open 3D sensor
roslaunch roch_objects_detection standalone.launch
```
# Subscribe Topics

 * topic: /camera/image_raw
 
   * type: sensor_msgs::ImageConstPtr
  
   * descriptions: Input image topic.
  
# Publish Topics

 * topic: /detection/objects
 
   * type: sensor_msgs::Image
  
   * descriptions: modifid input image with mark objects detected.
  
