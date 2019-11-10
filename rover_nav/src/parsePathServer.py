#!/usr/bin/python

# Description: Contains two services:
#                (1): Parses gpsGoals.txt, creates an array of poses and publishes it as a Path msg.
#                (2): Publishes a specific waypoint from the pose array as a PoseStamped msg.
#
# Authors: Jeremie Bourque
#
# Date created: 27-10-2019
# Date last updated: 10-11-2019

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
from rover_nav.srv import setWaypoint, setWaypointResponse

GPSGoalsFile = "src/rover_control/rover_nav/src/gpsGoals.txt"
poseArray = []

# Initialize the node and services.
def parsePathServer():
    rospy.init_node('parsePathServer')     
    s1 = rospy.Service('parsePath', parsePath, handleParsePath)
    s2 = rospy.Service('setWaypoint', setWaypoint, handleSetWaypoint)
    print "Ready"
    rospy.spin()
     
# Handle parsePath service requests 
def handleParsePath(req):
    print "Parsing gpsGoals.txt"
    path = Path()
    path.header = makeHeader("wgs84")
    parseGPSGoals()
    path.poses = poseArray
    pubPath.publish(path)
    return "Parsing completed"

# Handle setWaypoint service requests
def handleSetWaypoint(req):
    global poseArray
    print "Setting waypoint #%d" % (req.waypointNumber)
    if len(poseArray) == 0:  # No path published, return error
        print "Path empty, make sure you published the path first"
        return "Failed, no path published"
    pubWaypoint.publish(poseArray[req.waypointNumber-1])
    print str(poseArray[req.waypointNumber-1])
    print "Waypoint set"
    return "Success"

# Parses the GPS goals text file to get array of gps goals.
def parseGPSGoals():
    global poseArray
    with open(GPSGoalsFile) as goals:
        lines = goals.readlines()
        print("--Waypoints--")
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

# Makes the header message needed for the other messages.
def makeHeader(frame_id):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    return header
    
if __name__ == '__main__':
    pubPath = rospy.Publisher('path_topic', Path, queue_size=10)
    pubWaypoint = rospy.Publisher('waypoint_topic', PoseStamped, queue_size=10)
    parsePathServer()

