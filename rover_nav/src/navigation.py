#!/usr/bin/python

# Description: This node generates Twist commands to guide the rover towards its goal. It needs a goal with wgs84
#              coordinates (lat/lon) and the origin of the map frame from the /local_xy_origin topic in order to work.
#              The origin is defined in the launch file and automatically published.
#
# Authors: Jeremie Bourque
#
# Date created: 16-12-2019
# Date last updated: 16-18-2019

import rospy
import math
import tf
from geopoint import distanceBetween2CoordsXY, Wgs84ToXY, distanceBetweenToPoints
from geometry_msgs.msg import PoseStamped, Twist


def handleGoal(data):
    global origin_pose
    currentGoal = data
    # Convert goal from wgs84 frame to map frame.
    (currentGoal.pose.position.x, currentGoal.pose.position.y) = Wgs84ToXY(currentGoal.pose.position.y, currentGoal.pose.position.x)

    goalReached = False
    rate = rospy.Rate(10)  # TODO review this value
    while not goalReached:
        # TODO: implement way to abort
        # TODO: Would it be simpler to broadcast the goal as a tf frame and directly lookup transform between rover and goal instead of calculation the transform manually?
        # Get transform between map and rover in the map frame.
        try:
            (trans, rot) = listener.lookupTransform('map', 'rover', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # Calculate the distance between the rover and the goal in the map frame
        distX = currentGoal.pose.position.x - trans[0]
        distY = currentGoal.pose.position.y - trans[1]
        theta = (tf.transformations.euler_from_quaternion(rot))[2]

        # Use the transformation matrix (calculated by hand) to convert the distance between the rover and goal in the map frame to the rover's frame
        x = (distX*math.cos(theta)) + (distY*math.sin(theta))
        y = (-distX*math.sin(theta)) + (distY*math.cos(theta))

        # TODO: As a user, I want to easily see the remaining distance to reach the goal.
        distanceToGoal = distanceBetweenToPoints(trans[0], trans[1], currentGoal.pose.position.x, currentGoal.pose.position.y)
        rospy.logwarn("Distance to goal: %f" % distanceToGoal)

        # If the rover is close enough the the goal, stop navigation.
        if distanceToGoal < 10:  # TODO: review this value
            goalReached = True
            # TODO: if we break instead of raising a flag, the last twist command won't be sent. Should we do this instead?
            rospy.loginfo("Goal Reached! Stopping navigation")

        # Convert to twist commands and publish
        cmd = Twist()
        cmd.linear.x = 0.5 * math.sqrt(x**2 + y**2) # The constant defines the aggressiveness of the command. (higher = more aggressive)
        cmd.angular.z = 0.75 * math.atan2(y, x)  # The constant defines the aggressiveness of the command. (higher = more aggressive)
        nav_cmd.publish(cmd)
        rate.sleep()
    rospy.loginfo("Navigation stopped")


if __name__ == '__main__':
    rospy.init_node('navigation')
    br = tf.TransformBroadcaster()
    origin_pose = rospy.wait_for_message('local_xy_origin', PoseStamped)
    listener = tf.TransformListener()
    nav_cmd = rospy.Publisher('nav_cmd', Twist, queue_size=10)  # TODO: change to a Command msg instead?
    rospy.loginfo("Navigation ready")
    rospy.Subscriber('waypoint_topic', PoseStamped, handleGoal)
    rospy.spin()
