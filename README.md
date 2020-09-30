# romap

ROS node that contains the UAV manager node for the coordination of a Mobile Manipulator and a UAV using a single haptic device.

This node implements the UAV code for:
* Teleoperating a mobile manipulator and a free-flying camera from a single haptic device, JA Claret, I Zaplana, L Basanez - 2016 IEEE International Symposium on Safety, Security, and Rescue Robotics (SSRR), 2016

This repo contains several nodes that allow the control of a UAV. In particular:
* [ardrone_key_teleop.cpp](https://github.com/joseparnau/romap/blob/master/src/ardrone_key_teleop.cpp) allows the command of a Parrot Ardrone using the keyboard.
* [tcp2cam_cube.cpp](https://github.com/joseparnau/romap/blob/master/src/tcp2cam_cube.cpp) computes the relative transformation of the UAV w.r.t. a cub of [markers](http://wiki.ros.org/ar_track_alvar).
* [ardrone_manager.cpp](https://github.com/joseparnau/romap/blob/master/src/ardrone_manager.cpp) contains the logic necessary to integrate the UAV within the teleoperation robot system.
