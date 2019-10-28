#!/usr/bin/python

# Description: Parses a text file containing a list of coordinates and makes an array of
#              poses to publish a path message.
#
# Authors: Jeremie Bourque
#
# Date: 27-09-2019
#

# gpsGoals.txt example:
#   #Example comment
#   45.50049,-73.62600 #example comment
#   45.50149,-73.62600
#   45.50249,-73.62800
#   45.50349,-73.62900

import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from rover_nav.srv import parsePath, parsePathResponse

GPSGoalsFile = "/home/jeremie/Rover/src/rover_control/rover_nav/src/gpsGoals.txt"

def parsePathServer():
    rospy.init_node('parsePathServer')     
    s = rospy.Service('parsePath', parsePath, handleParsePath)
    print "Ready"
    rospy.spin()
     
def handleParsePath(req):
    print "Parsing gpsGoals.txt"
    path = Path()
    path.header = makeHeader("wgs84")
    path.poses = parseGPSGoals()
    pub.publish(path)
    return "Parsing completed"

# Parses the GPS goals text file to get array of gps goals.
def parseGPSGoals():
    poseArray = []
    with open(GPSGoalsFile) as goals:
        lines = goals.readlines()
        print("--GPS Goals--")
        for line in lines:
            # Remove comment part of the line.
            lineWithoutComment = line[0:line.find("#")]         
            if len(lineWithoutComment) != 0:
                coordinate = lineWithoutComment.split(",")
                print coordinate
                try:         
                    lat = float(coordinate[0])
                    lon = float(coordinate[1])
                    pose = PoseStamped()
                    pose.header = makeHeader("wgs84")
                    pose.pose.position.x = lon
                    pose.pose.position.y = lat
                    poseArray.append(pose)
                except ValueError:
                    print "Not a float, ignoring line."
    return poseArray           

# Makes the header message needed for the other messages.
def makeHeader(frame_id):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    return header
    
if __name__ == '__main__':
    pub = rospy.Publisher('path_topic', Path, queue_size=10)
    parsePathServer()

