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
        self.enable_button = rospy.get_param("~enable_button", 4)
        self.enable_turbo_button = rospy.get_param("~enable_turbo_button", 5)
        self.linear_scaling = rospy.get_param("~scale_linear", 0.0)
        self.linear_scaling_turbo = rospy.get_param("~scale_linear_turbo", 0.0)
        self.angular_scaling = rospy.get_param("~scale_angular", 0.0)
        self.axis_linear = rospy.get_param("~axis_linear", 1)
        self.axis_angular = rospy.get_param("~axis_angular", 0)

   def joyCB(self, data):
        twist = Twist()
        twist.linear.x = data.axes[self.axis_linear]
        twist.angular.z = data.axes[self.axis_angular]

        if data.buttons[self.enable_button] == 1: #Verify the dead man button is active (LB)
            rospy.set_param("~scale_angular", 1.0)
            self.angular_scaling = rospy.get_param("~scale_angular") # updates the scaling
            twist.angular.z *= self.angular_scaling #Both the turbo mode and normal mode share the same angular speed

            if data.buttons[self.enable_turbo_button] == 1:
                rospy.set_param("~scale_linear_turbo", 4.0)
                self.linear_scaling_turbo = rospy.get_param("~scale_linear_turbo")
                twist.linear.x *= self.linear_scaling_turbo
                self.cmd_pub.publish(twist)
            else:
                rospy.set_param("~scale_linear", 1.0)
                self.linear_scaling = rospy.get_param("~scale_linear")
                twist.linear.x *= self.linear_scaling 
                self.cmd_pub.publish(twist)
        else:
            self.linear_scaling = rospy.set_param("~scale_linear", 0.0) #If no deadman is active, sets all param to 0 and twists are sent at 0
            self.linear_scaling_turbo = rospy.set_param("~scale_linear_turbo", 0.0)
            self.angular_scaling = rospy.get_param("~scale_angular", 0.0)
            twist.angular.z = 0.0
            twist.linear.x = 0.0
            self.cmd_pub.publish(twist)
        

if __name__ == '__main__':
    teleop_joystick = TeleopJoystick()
    rospy.loginfo("teleop_joystick ready")
    rospy.spin()

    