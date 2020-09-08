#!/usr/bin/env python

import rospy
import socket
from sensor_msgs.msg import NavSatFix

class GPSToAntenna: 
    HOST = '10.42.0.30'  # server hostname
    PORT = 65432
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def __init__(self):
        sub = rospy.Subscriber("fix", NavSatFix, self.fixCB)
        self.s.connect((self.HOST, self.PORT))  # TODO: Verify that we can connect before attempting

    def fixCB(self, msg):
        coord = "%f;%f" % (msg.latitude, msg.longitude)
        print(coord)
        self.s.send(coord)

    def closeConnection(self):
        self.s.close()  # TODO: When should the connection be closed?


if __name__ == "__main__":
    rospy.init_node("gps_to_antenna")
    gpsToAntenna = GPSToAntenna()
    rospy.spin()
