#!/usr/bin/python

# Description: Node that merges the gps fix with the IMU's heading to publish the rover's tf frame.
#
# Authors: Jeremie Bourque
#
# Date created: 18-01-2020
# Date last updated: 18-01-2020


import rospy
import tf
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from geopoint import Wgs84ToXY


currentLat = 0
currentLon = 0
currentHeading = 0


def updatePosition(data):
    currentLat = data.latitude
    currentLon = data.longitude


def updateHeading(data):
    pass
    #currentHeading = data.heading TODO


def sendTransform():
    (currentX, currentY) = Wgs84ToXY(currentLat, currentLon)
    # Send transform of rover in the map frame
    br.sendTransform((currentX, currentY, 0),
                     tf.transformations.quaternion_from_euler(0, 0, currentHeading),
                     rospy.Time.now(),
                     'rover',
                     'map')


if __name__ == '__main__':
    rospy.init_node('rover_pose')
    br = tf.TransformBroadcaster()
    rospy.Subscriber('fix', NavSatFix, updatePosition)
    # rospy.Subscriber('heading', IMU, updateHeading) TODO
    origin_pose = rospy.wait_for_message('local_xy_origin', PoseStamped)
    rospy.loginfo("rover_pose ready")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        sendTransform()
        rate.sleep()
    rospy.spin()
