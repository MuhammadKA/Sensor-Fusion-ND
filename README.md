# Udacity Sensor Fusion Nanodegree 

## Course Notes 
https://www.notion.so/Sensor-Fusion-Nano-degree-ec8f58bb4d24439fa5c1b912719ddda2

### Welcome to the Sensor Fusion course for self-driving cars.

In this course we will be talking about sensor fusion, whch is the process of taking data from multiple sensors and combining it to give us a better understanding of the world around us. we will mostly be focusing on two sensors, lidar, and radar. By the end we will be fusing the data from these two sensors to track multiple cars on the road, estimating their positions and speed.

**Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Also we can tell a little bit about the object that was hit by measuring the intesity of the returned signal. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. While lidar sensors gives us very high accurate models for the world around us in 3D, they are currently very expensive, upwards of $60,000 for a standard unit.

**Radar** data is typically very sparse and in a limited range, however it can directly tell us how fast an object is moving in a certain direction. This ability makes radars a very pratical sensor for doing things like cruise control where its important to know how fast the car infront of you is traveling. Radar sensors are also very affordable and common now of days in newer cars.

**Sensor Fusion** by combing lidar's high resoultion imaging with radar's ability to measure velocity of objects we can get a better understanding of the sorrounding environment than we could using one of the sensors alone.

# Projects

## Lidar Obstacle Detection

![Lidar Clustering](https://github.com/MuhammadKA/Sensor-Fusion-ND/blob/master/1-%20Lidar%20Obstacle%20Detection/ObstacleDetectionFPS.gif)

## 2D Feature Tracking

![2D Feature Tracking](https://github.com/MuhammadKA/Sensor-Fusion-ND/blob/master/2-%20Camera:%202D%20Feature%20Matching/images/keypoints.png)

## 3D Object Tracking

![3D Object Tracking](https://github.com/MuhammadKA/Sensor-Fusion-ND/blob/master/2-%20Camera:%203D%20Object%20Tracking/course_code_structure.png)

## RADAR Target Generation and Detection

![RADAR Target Generation and Detection](https://github.com/yosoufe/SFND_RADAR/raw/master/project_layout.png)

## Unscented Kalman Filter

![Unscented_Kalman_Filter](https://github.com/MuhammadKA/Sensor-Fusion-ND/blob/master/4-%20Unscented%20Kalman%20Filter/media/ukf_highway_tracked.gif)


