#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, Quaternion
from math import sqrt, atan2, pi
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from collections import deque
from threading import Lock

class GpsHeading():
    def __init__(self):
        # ROS stuff
        rospy.init_node("gps_heading", anonymous = True)
        self.gps_sub = rospy.Subscriber('/odometry/gps', Odometry, self.gpsCB)
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imuCB)
        self.heading_pub = rospy.Publisher('/gps_heading', Imu, queue_size=1)

        # Parameters
        self.publishing_rate = 10  # Hz
        self.min_distance = 3  # TODO: ROS parameter
        self.base_frame = "base_link"
        self.gps_positions_queue_length = 5  # Newest positions are added on the left, oldest positions are on the right
        self.orientation_covariance = [0.1, 0, 0, \
                                       0, 0.1, 0, \
                                       0, 0, 0.1]  # TODO: use better covariance values

        # Member variables
        self.gps_position_queue = deque(maxlen=self.gps_positions_queue_length)
        self.latest_gps_heading = 0
        self.heading_msg = Imu()

        self.got_first_imu_orientation = False
        self.previous_imu_orientation = Quaternion()
        self.relative_yaw = 0
        self.latest_imu_euler_orientation = [0, 0, 0]

        self.mutex = Lock()  # Mutex is used because the two callbacks run in their own thread but they use shared data

        # Initialization process
        self.heading_msg.header.frame_id = self.base_frame
        self.heading_msg.orientation_covariance = self.orientation_covariance

    def run(self):
        try:
            r = rospy.Rate(self.publishing_rate)
            while not rospy.is_shutdown():
                self.mutex.acquire()
                self.heading_msg.header.stamp = rospy.Time.now()
                self.heading_pub.publish(self.heading_msg)
                self.mutex.release()
                r.sleep()
        except rospy.exceptions.ROSInterruptException:
            pass
            
    def imuCB(self, msg):
        if not self.got_first_imu_orientation:
            self.previous_imu_orientation = msg.orientation
            self.got_first_imu_orientation = True
            return

        # Get yaw difference between current yaw and previous yaw
        previous_imu_quaternion_orientation = [self.previous_imu_orientation.x, self.previous_imu_orientation.y, \
                                              self.previous_imu_orientation.z, self.previous_imu_orientation.w]
        current_imu_quaternion_orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

        previous_imu_euler_orientation = euler_from_quaternion(previous_imu_quaternion_orientation)
        current_imu_euler_orientation = euler_from_quaternion(current_imu_quaternion_orientation)

        self.relative_yaw += limit_angles_pi(current_imu_euler_orientation[2] - previous_imu_euler_orientation[2])
        self.relative_yaw = limit_angles_pi(self.relative_yaw)
        absolute_yaw = limit_angles_pi(self.relative_yaw + self.latest_gps_heading)

        absolute_quaternion_orientation = quaternion_from_euler(current_imu_euler_orientation[0], current_imu_euler_orientation[1], absolute_yaw)

        self.mutex.acquire()
        self.latest_imu_euler_orientation = current_imu_euler_orientation
        self.heading_msg.orientation.x = absolute_quaternion_orientation[0]
        self.heading_msg.orientation.y = absolute_quaternion_orientation[1]
        self.heading_msg.orientation.z = absolute_quaternion_orientation[2]
        self.heading_msg.orientation.w = absolute_quaternion_orientation[3]
        self.mutex.release()
        # Rest of IMU msg is left empty since we only care about the orientation

        self.previous_imu_orientation = msg.orientation


    def gpsCB(self, msg):
        # A queue is used instead of only saving one previous position because if we only save one, the robot has to
        # travel the entire threshold distance before a new heading can be calculated. If we save multiple previous 
        # positions in a queue, we can calculate more frequently because one of the few previous positions is bound
        # to be the threshold distance away already.
        for position in self.gps_position_queue:
            # Starting from the most recent previous position, find a previous position that is
            # farther than the minimum distance threshold
            dx = msg.pose.pose.position.x - position.x
            dy = msg.pose.pose.position.y - position.y
            distance = sqrt(dx**2 + dy**2)

            if distance > self.min_distance:
                self.mutex.acquire()
                self.latest_gps_heading = atan2(dy, dx) 
                quaternion_orientation = quaternion_from_euler(self.latest_imu_euler_orientation[0],  self.latest_imu_euler_orientation[1], \
                                                               self.latest_gps_heading)

                self.heading_msg.orientation.x = quaternion_orientation[0]
                self.heading_msg.orientation.y = quaternion_orientation[1]
                self.heading_msg.orientation.z = quaternion_orientation[2]
                self.heading_msg.orientation.w = quaternion_orientation[3]      
                # Rest of IMU msg is left empty since we only care about the orientation

                self.relative_yaw = 0  # Reset relative yaw since a new gps heading has arrived
                print("new gps heading")  
                self.mutex.release()

                break
        self.gps_position_queue.appendleft(msg.pose.pose.position)


def limit_angles_pi(angle):
    # Limit an angle in radians to -pi : pi
    if angle > pi:
        angle = (angle%(2*pi)) - 2*pi
    elif angle < -pi:
        angle = 2*pi + (angle%(-2*pi))
    return angle


if __name__ == '__main__':
    gps_heading = GpsHeading()
    rospy.loginfo("gps_heading ready")
    gps_heading.run()

    