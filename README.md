# SLAM
In this repo, I will try to implement different SLAM algorithms on a simple differential drive robot simulation, with Webots.

### 1) Measuring pose from wheel tick (Odometry)
Based on the differential drive model of turtlebot, and counting the wheel displacement at each measurement step, we can estimate the pose of the robot. Note that due to small angle approximation in the turn model and due to wheel slip, the pose error is accumulating when we use only wheel encoder as sensors.
