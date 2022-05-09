# Shepherding with a Misty
## A Final Project for Probabilistic Robotics at Tufts University, Spring 2022
Authored by Emma Bethel, Allison Moore, and Ray Rogers

## Overview

Due to pandemic-related issues, we were unable to combine all of our different pieces of the code and get them running in tandem on the robot. This codebase thus contains several disconnected piecs of the robot code that were intended to be meshed together-- a Kalman filter, in the [kalman](./kalman) directory, a simulation of the control algorithm, in [simulation/submission](./simulation/submission) and some code for path planning/approaching a vision target using Misty SLAM capabilities in [moving_around](./moving_around).

## Individual Contributions 

### Emma

Emma did most of the implementation of the math from the paper we based our algorithm on, including working out why and how to flip those ua vectors to being orthogonal to the ones described in the paper (after some brainstorming with Allison).

Emma also designed and wrote pretty much everything in [moving_around](./moving_around/) in order to get the robot to sense a vision target, estimate its distance away from it, and integrate that knowledge with the built-in SLAM capabilities of the Misty to generate and follow a path toward what it believes to be the most likely position of the target. For that portion, the SLAM usage was based off of some experimenetation and testing done by Allison with the Misty's web interface (as discussed in her section below). 

### Ray

### Allison -- lmk if i accidentally take credit for anything you did etc

Allison primarily worked on the initial implementation of SLAM mapping, our Kalman Filter face detection implementation, and our simulation. Allison also experienced with HTTP distance requests to the Misty, but not much came of this because she got COVID.

For SLAM mapping, Allison used the Misty command center to Map the robotics teaching lab and track the position of the Misty as it moved about the space. 

For the Kalman Filter, Allison implemented a facial distance detection algorithm alongside a kalman filter algorithm to later be integrated on the herd to keep track of the flock probabilistically. 

Within the simulation specifically, Allison worked primarily on the main.py simulation compilation with OpenCV and the flock avoidance function within the robot.py class. 

In a more general sense, Allison worked with Emma to understand the Math behind our simulation and break down down how to translate the algorithm intoduced in our paper into our simulation. When the initial simulation algorithm didn't work, Allison tested potential alternatives to the original herding algorithm with Emma. 

## References

For the Kalman Filter, we referenced the object tracking tutorial on pysource and also used the opencv Kalman Filter Library. For the face detection portion of our algorithm, we referenced a geekforgeeks tutorial. 
https://pysource.com/2021/10/29/kalman-filter-predict-the-trajectory-of-an-object/
https://docs.opencv.org/4.x/dd/d6a/classcv_1_1KalmanFilter.html 
https://www.geeksforgeeks.org/realtime-distance-estimation-using-opencv-python/ 

For the estimation of vision target distance from its size in the image, we used the method described in this article: https://medium.com/analytics-vidhya/how-to-track-distance-and-angle-of-an-object-using-opencv-1f1966d418b4

Our main algorithm (as seen in [simluation](./simulation/submission)) was largely based off of the one described in this paper: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9173524&tag=1 




