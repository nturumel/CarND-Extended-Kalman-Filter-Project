# Extended Kalman Filter 
## Code
Code can be found in the src directory. 
The following source files are the main players:

#### 1. FusionEKF.cpp
This is used for initialising the variables needed for the actual kalman filter.
The variables initialised include:
##### state x, prediction F, measurement noise H, prediciton noise P

#### 2. KalmanFilter.cpp
This class implements the predict and update function, care is taken to handle the radar data which is in polar coordinates. 
Because lidar uses linear equations, the update step will use the basic Kalman filter equations. On the other hand, radar uses non-linear equations, so the update step involves linearizing the equations with the Jacobian matrix. 

#### 3. Tools.cpp
This implements functions to calculate root mean squared error and the Jacobian matrix.

## Build Instructions
The main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

## Results
My px, py, vx, and vy RMSE should be less than or equal to the values [.11, .11, 0.52, 0.52].
The results look something like this.


[![Watch the video](https://i.imgur.com/vKb2F1B.png)](https://youtu.be/edaXaYf95ls)

