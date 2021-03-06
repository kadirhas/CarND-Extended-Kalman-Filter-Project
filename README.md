# Extended Kalman Filter Project
This is the first project of the second term on Udacity Self-Driving Car Engineer Nanodegree Program.

In this project, a simulated radar and lidar data is fused using an Extended Kalman Filter. The project utilizes Udacity simulator to show the results of the filter, which requires uWebSocketIO to provide data transfer between the simulation and the code.

[//]: # (Image References)

[image1]: ./Docs/close_data1.png "Dataset 1 results"
[image2]: ./Docs/data1_all.png "Complete path of Dataset 1"
[image3]: ./Docs/data2_all.png "Complete path of Dataset 2"

## Building the project
The main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake .. && make
4. ./ExtendedKF

## Results
The code filters well both of the provided datasets. The RMSE is calculated for both datasets can be seen on the images below:

![alt text][image1]
![alt text][image2]
![alt text][image3]

## Discussion
For a single object tracking, this project is a simple solution. However, any correlations between the sensors are not considered since it is only using Kalman Filter. To solve this, more advanced fusion method such as Covariance Intersection or Information Matrix Fusion might be used. This project also considers the number of objects that is tracked is known and the measurements are not cluttered. As a further improvements, a track management algorithm (to determine the number of tracks) and an association algorithm (to determine which measurements belongs to which tracked object) could be developed. It is also assumed that the tracked objects does not make sudden maneuvers that doesn't fit to constant velocity model. For these situations Multi model algorithms could be developed. 