 # Writeup: Track 3D-Objects Over Time

Please use this starter template to answer the following questions:

### 1. Write a short recap of the four tracking steps and what you implemented there (filter, track management, association, camera fusion). Which results did you achieve? Which part of the project was most difficult for you to complete, and why?

Step 1. Tracking with Extended Kalman Filter (EKF): I implemented a non-linear Kalman filter with 6 dimensions. The states were the position (x, y, z) and the velocity (vx, vy, vz). The filter is able to combine noisy measurements from multiple sensors (camera, lidar) and fuse them with a model of the system (vehicle motion). This allows the filter to:
- Predict the vehicle's future state based on its dynamics
- Correct the prediction using incoming sensor data
- Maintain a robust estimate of position and velocity over time, even in the presence of noise and uncertainty

The model used was a constant velocity model:
```
F = [1., 0., 0., self.dt, 0., 0.],
    [0., 1., 0., 0., self.dt, 0.],
    [0., 0., 1., 0., 0., self.dt],
    [0., 0., 0., 1., 0., 0.],
    [0., 0., 0., 0., 1., 0.],
    [0., 0., 0., 0., 0., 1.]]
```

Step 2. Track Management: Track management refers to the process of maintaining a set of target tracks over time in a dynamic environment. Each track represents a hypothesized object (in this case, vehicle) being monitored. I implemented a module responsible for track management (new tracks creation, deletion of old tracks based on sensor measurements). This included calculation of track score to confirm likely tracks and reject of implausible tracks / false positives. A moving window of 6 frames was used, each time a measurement is associated with a track, the track score is increased of +1/window. If in a frame the object is not present the score is decreased. The score is held constant of the predicted position is outside the sensor's fov, in this case the track is delected after some time by checking the state covariance matrix P.

Step 3. Data Association: In the data association module, all new measurements are assigned to a track. The data association algorithm decides whether a given measurement was generated from an existing track (and in case it updates the corresponding track), or whether it represents a newly-detected track (e.g., a car that has just entered the fov). Using an association matrix, the algorithm assigns a track the closest measurement using the Mahalanobis distance, which accounts for uncertainty in both the prediction and the measurement. The algorithm works assuming that each track generates at most one measurement, and each measurement originates from at most one track.

Step 4. Sensor Fusion: Allows to fuse measurements coming from different sensors, namely camera and lidar. A transformation is needed to translate from vehicle coordinates to sensor coordinates to compute the prediction error. For camera the measurement model h(x) is non linear. A method that checks whether an object can be seen by the sensor or is outside the field of view was also implemented.

### 2. Do you see any benefits in camera-lidar fusion compared to lidar-only tracking (in theory and in your concrete results)? 


### 3. Which challenges will a sensor fusion system face in real-life scenarios? Did you see any of these challenges in the project?


### 4. Can you think of ways to improve your tracking results in the future?

