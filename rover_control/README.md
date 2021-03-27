# rover_control

## gps_heading
gps_heading is a node that outputs the robot's global orientation by fusing the GPS odometry and the IMU orientation. <br>
The GPS heading can be found by calculating the yaw angle of a vector going from a previous GPS position to the current one. This heading however is only accurate if the distance vector is long enough and if the robot has been moving in a straight line. A GPS heading is considered accurate if the it's difference from the last heading is less than the specified epsilon value.
Because of that, accurate GPS headings are infrequent and can't be used alone. The solution is to use the relative IMU rotation in between GPS heading hits. That way we get a high frequency driftless global heading.

### Subscribed topics
* odometry/gps(nav_msgs/Odometry) GPS odometry odometry message
* imu/data(sensor_msgs/Imu) IMU message

### Published topics
* gps_heading(sensor_msgs/Imu) Fused orientation robot orientation

### Parameters
* ~publishing_rate(int, default: 10) Rate in Hz to publish the fused orientation message
* ~min_distance(float, default: 3) Minimum distance vector length to calculate GPS heading
* ~base_frame(string, default: "base_link") Name of the robot's base frame
* ~gps_positions_queue_length(int, default: 5) Number of the most recent GPS positions to keep in memory
* ~gps_heading_epsilon(float, default: 0.0523599) Maximum difference in radians between the current gps heading and previous gps heading to consider the heading converged