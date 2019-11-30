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

import os
import rospy
from geopoint import Geopoint
from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from rover_nav.srv import createPath, createPathResponse
from rover_nav.srv import setWaypoint, setWaypointResponse

GPSGoalsFile = "gpsGoals.txt"


# Initialize the node and services.
def pathMgr():
    rospy.init_node('pathMgr')
    s1 = rospy.Service('createPath', createPath, handleCreatePath)
    s2 = rospy.Service('setWaypoint', setWaypoint, handleSetWaypoint)
    print "Ready"
    rospy.spin()
     
# Handle createPath service requests
def handleCreatePath(req):
    print "Creating path"
    print(os.getcwd())
    path = Path()
    path.header.frame_id = "wgs84"
    path.header.stamp = rospy.Time.now()
    geopoints = parseGPSGoals()
    for gp in geopoints:
        path.poses.append(gp.poseStamped())
    pubPath.publish(path)
    return "Parsing completed"

# Handle setWaypoint service requests
def handleSetWaypoint(req):
    print "Setting waypoint #%d" % (req.waypointNumber)
    geopoints = parseGPSGoals()
    if len(geopoints) == 0:  # No path published, return error
        print "Failed, path empty"
        return "Failed, path empty"
    try:
        pubWaypoint.publish(geopoints[req.waypointNumber-1].poseStamped())
    except IndexError:
        print "waypoint number out of bounds"
    print str(geopoints[req.waypointNumber-1])
    print "Waypoint set"
    return "Success"

# Parses the GPS goals text file to get array of gps goals.
def parseGPSGoals():
    count = 1
    geopoints = []
    with open(GPSGoalsFile) as goals:
        lines = goals.readlines()
        print("--Waypoints--")
        for line in lines:
            # Remove comment part of the line.
            lineWithoutComment = line[0:line.find("#")]
            if len(lineWithoutComment) != 0:
                if lineWithoutComment[-1] == ' ':
                    lineWithoutComment = lineWithoutComment[:-1]
                coordinate = lineWithoutComment.split(",")
                print str(count) + " " + str(coordinate)
                count += 1
                try:         
                    lat = float(coordinate[0])
                    lon = float(coordinate[1])
                    gp = Geopoint()
                    gp.lon = lon
                    gp.lat = lat
                    geopoints.append(gp)
                except ValueError:
                    print "Not a float, ignoring line."
    return geopoints

    
if __name__ == '__main__':
    pubPath = rospy.Publisher('path_topic', Path, queue_size=10)
    pubWaypoint = rospy.Publisher('waypoint_topic', PoseStamped, queue_size=10)
    pathMgr()

