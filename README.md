# Extended Kalman Filter 
## Code
Code can be found in the src directory. 
The following source files are the main players:

## 1. FusionEKF.cpp
This is used for initialising the variables needed for the actual kalman filter.
The variables initialised include:
### state x
### prediction F
### measurement noise H
### prediciton noise P

## 2. KalmanFilter.cpp
This class implements the predict and update function, care is taken to handle the radar data which is in polar coordinates. 
Because lidar uses linear equations, the update step will use the basic Kalman filter equations. On the other hand, radar uses non-linear equations, so the update step involves linearizing the equations with the Jacobian matrix. 

## 3. Tools.cpp
This implements functions to calculate root mean squared error and the Jacobian matrix.
