# rgbd-object-tracking
**MSc Thesis work**

For robotic applications, we almost always need the exact spatial position and orientation of the object to be manipulated to properly handle the robot. There are a number of methods known for object recognition and its position and orientation, which usually require RGB image-based machine vision, but facilitates the process if the image information includes depth data. However, image-based recognition can be difficult if an item is covered by another object. The relative movement of the camera and object may also result from the camera being fixed to the robot arm.

During my master thesis work I created an experimental configuration, familiarized myself with using ROS framework, C++ specific libraries (Boost, Point CLoud Library, Eigen) depth-image-based point clouds, pointcloud object-registration, segmentation, implementation of a real-time object-tracking algorithm and testing the algorithm using the 6-DOF Mitsubishi industrial robotic arm.

