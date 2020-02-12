#!/usr/bin/env python
import roslib
import rospy
from rover_udes.msg import Command
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


MAX_VALUE = 100


class CmdMux:
    def __init__(self):
        # By default, commands come from joystick
        self.gui_control_on = False
        self.nav_control_on = False
        self.right_cmd = 0.0
        self.left_cmd = 0.0
        
        rospy.init_node("cmd_mux")
        rospy.on_shutdown(self.on_shutdown)
        self.gui_sub = rospy.Subscriber('/gui_cmd', Command, self.gui_cmd_callback)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.nav_sub = rospy.Subscriber('/nav_cmd', Twist, self.nav_cmd_callback)

        self.cmd_pub = rospy.Publisher('mux_cmd', Command, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / 10.0), self.publish_cmd)

    def gui_cmd_callback(self, cmd):
        self.gui_control_on = cmd.is_active
        if self.gui_control_on:
            self.right_cmd = cmd.right
            self.left_cmd = cmd.left

    def joy_callback(self, joy):
        # Left bumper acts as dead-man switch
        if (joy.buttons[4] == 1) and not self.gui_control_on:
            self.left_cmd = joy.axes[1]
            self.right_cmd = joy.axes[4] 
        else:
            self.left_cmd = 0
            self.right_cmd = 0

    def nav_cmd_callback(self, cmd):
        uncapped_right, uncapped_left = twistToTank(cmd.linear.x, cmd.angular.z)
        self.right_cmd, self.left_cmd = limitCmd(uncapped_right, uncapped_left)

    def publish_cmd(self, event):
        cmd = Command()
        cmd.is_active = True
        cmd.right = self.right_cmd
        cmd.left = self.left_cmd
        self.cmd_pub.publish(cmd)

    def on_shutdown(self):
        pass


# Convert twist commands to tank (left and right) command
def twistToTank(linear, angular):
    right_cmd = linear - angular
    left_cmd = linear + angular
    return right_cmd, left_cmd


# Limits right and left command from 0 to 100 while keeping the difference between the two proportional
def limitCmd(right, left):
    if right > MAX_VALUE or left > MAX_VALUE:
        if right >= left:
            capped_right = 100
            capped_left = (left/right) * 100
        else:
            capped_left = 100
            capped_right = (right / left) * 100
        rospy.logwarn("right = %f  left = %f" % (right, left))
        return capped_left, capped_right
    return right, left


if __name__ == '__main__':
    try:
        mux = CmdMux()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
