#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class TeleopJoystick():
    def __init__(self):
        rospy.init_node("teleop_joystick")
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joyCB)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def joyCB(self, data):
        # TODO: your code here, remove pass
        pass

if __name__ == '__main__':
    teleop_joystick = TeleopJoystick()
    rospy.loginfo("teleop_joystick ready")
    rospy.spin()

    