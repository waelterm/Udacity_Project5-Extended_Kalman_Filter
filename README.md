# Udacity_Project5-Extended_Kalman_Filter
This repository includes my code for the Extended Kalman Filter Project of the Udacity Self-Driving Car Nanodegree.

# Description:
In this project I implemented an Extended Kalman Filter for Tracking of a vehicle based on lidar and radar data.
A constant velocity model has been used for the Prediction loop. Two different measurement loops were used:
1. Lidar: The Measurement function for the lidar measurement is linear. Therefore a standard Kalman Filter could be used.
2. Radar: Transforming the radar signal from cartesian into polar coordinates is a nonlinear operation. The measurement function was approximated using the Jacobian Matrix in an Extended Kalman Filter.

# Testing
This project was tested using the [EKF Simulator from Udacity](https://github.com/udacity/self-driving-car-sim/releases).

# Results
The project was evaluated by calculating the Root Mean Squared Error of the x-position, y-position, x-velocity, y-velocity. 
These were the results:

I started by implementing the lidar measurement loop first. This already gave some decent results:
With Lidar only: 
		 Dataset 1: 0.2249, 0.2186, 3.7413, 3.3195
     
I then added the Extended Kalman Filter for the Radar Measurements. This improved the results:
With Radar and Lidar:
		Dataset 1: 0.0973, 0.0855, 0.4513, 0.4399
		Dataset 2: 0.0726, 0.0967, 0.4579, 0.4966

  
# Implementation Notes
I used Visual Studio on Windows to write and format the C++ code. I also ran the Simulator on Windows.
I compiled and ran my code using the Windows Subsystem for Linux (Ubuntu 16.04 WSL). 
