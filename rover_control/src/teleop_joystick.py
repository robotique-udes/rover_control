#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class TeleopJoystick():
    def __init__(self):
        rospy.init_node("teleop_joystick", anonymous = True)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joyCB)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)

        #Getting parameters initialized at the launch file
        if rospy.has_param("joy/enable_button"):
           self.enable_button = rospy.get_param("joy/enable_button")
        if rospy.has_param("joy/enable_turbo_button"):
           self.enable_turbo_button = rospy.get_param("joy/enable_turbo_button")
        if rospy.has_param("joy/scale_linear"):
           self.linear_scaling = rospy.get_param("joy/scale_linear")
        if rospy.has_param("joy/scale_linear_turbo"):
           self.linear_scaling_turbo = rospy.get_param("joy/scale_linear_turbo")
        if rospy.has_param("joy/scale_angular"):
           self.angular_scaling = rospy.get_param("joy/scale_angular")
        if rospy.has_param("joy/axis_linear"):
           self.axis_linear = rospy.get_param("joy/axis_linear")
        if rospy.has_param("joy/axis_angular"):
           self.axis_angular = rospy.get_param("joy/axis_angular")

    def joyCB(self, data):
        twist = Twist()
        twist.linear.x = data.axes[self.axis_linear]
        twist.angular.z = data.axes[self.axis_angular]

        if data.buttons[self.enable_button] == 1: #Verify the dead man button is active (LB)
            twist.angular.z *= self.angular_scaling #Both the turbo mode and normal mode share the same angular speed
            
            if data.buttons[self.enable_turbo_button] == 1:
                twist.linear.x *= self.linear_scaling_turbo
                self.cmd_pub.publish(twist)
            else:
                twist.linear.x *= self.linear_scaling 
                self.cmd_pub.publish(twist)
        

if __name__ == '__main__':
    teleop_joystick = TeleopJoystick()
    rospy.loginfo("teleop_joystick ready")
    rospy.spin()

    