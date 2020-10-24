#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class TeleopJoystick():
    def __init__(self):
        rospy.init_node("teleop_joystick", anonymous = True)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joyCB)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)

    def joyCB(self, data):
        twist = Twist()
        twist.linear.x = data.axes[1]
        twist.linear.y = data.axes[0]

        rospy.loginfo("twist.linear.x: %f ; twist.linear.y %f", twist.linear.x, twist.linear.y)
        self.cmd_pub.publish(twist)

if __name__ == '__main__':
    teleop_joystick = TeleopJoystick()
    rospy.loginfo("teleop_joystick ready")
    rospy.spin()

    