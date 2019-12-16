#!/usr/bin/python
# Description: Geopoint object (lat,lon,description) along with other useful geographic functions
#
# Authors: Jeremie Bourque
#
# Date created: 30-11-2019
# Date last updated: 01-12-2019

import rospy
import math
from geometry_msgs.msg import PoseStamped, Pose
from visualization_msgs.msg import Marker, MarkerArray
from geographiclib.geodesic import Geodesic

# Calculates distance between two points in meters
# Slightly modified from http://wiki.ros.org/gps_goal
def distanceBetween2Coords(point1, point2):
    # Calculate distance and azimuth between GPS points
    geod = Geodesic.WGS84  # define the WGS84 ellipsoid
    g = geod.Inverse(point1.lat, point1.lon, point2.lat, point2.lon)  # Compute several geodesic calculations between two GPS points
    hypotenuse = distance = g['s12']  # access distance
    return hypotenuse


# Calculates distance between two points in the X and Y directions in meters
# Slightly modified from http://wiki.ros.org/gps_goal
def distanceBetween2CoordsXY(point1, point2):
    # Calculate distance and azimuth between GPS points
    geod = Geodesic.WGS84  # define the WGS84 ellipsoid
    g = geod.Inverse(point1.lat, point1.lon, point2.lat, point2.lon)  # Compute several geodesic calculations between two GPS points
    hypotenuse = distance = g['s12']  # access distance
    azimuth = g['azi1']

    # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) by finding side lengths of a right-angle triangle
    # Convert azimuth to radians
    azimuth = math.radians(azimuth)
    x = adjacent = math.cos(azimuth) * hypotenuse
    y = opposite = math.sin(azimuth) * hypotenuse

    return x, y


class Geopoint(object):
    def __init__(self, lat=200, lon=200, description=""):  # Note: 200,200 represents an invalid coordinate
        self.lat = lat
        self.lon = lon
        self.description = description

    def getLat(self):
        return self.lat

    def getLon(self):
        return self.lon

    def getDescription(self):
        return self.description

    def setLat(self, newLat):
        self.lat = newLat

    def setLon(self, newLon):
        self.lon = newLon

    def setDescription(self, newDescription):
        self.description = newDescription

    # Get a Geopoint with coordinates converted from rad to deg
    def radToDeg(self):
        geopointDeg = Geopoint()
        geopointDeg.lat = math.degrees(self.lat)
        geopointDeg.lon = math.degrees(self.lon)
        return geopointDeg

    # Get a Geopoint with coordinates converted from rad to deg
    def degToRad(self):
        geopointRad = Geopoint()
        geopointRad.lat = math.radians(self.lat)
        geopointRad.lon = math.radians(self.lon)
        return geopointRad

    # Returns distance in meters from self to a target point
    def distanceTo(self, target):
        return distanceBetween2Coords(self, target)

    # Returns distance in meters from self to a target point in the X and Y directions
    def distanceToXY(self, target):
        return distanceBetween2CoordsXY(self, target)

    # Creates a PoseStamped message
    def pose(self):
        pose = Pose()
        pose.position.x = self.lon
        pose.position.y = self.lat
        return pose

    # Creates a PoseStamped message
    def poseStamped(self, frame_id):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = frame_id
        pose.pose.position.x = self.lon
        pose.pose.position.y = self.lat
        return pose

    # Creates a marker message
    def marker(self, frame_id, waypointNum, type, color):
        # type = Marker().TYPE
        # refer to message definition for list of types
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "Current goal: waypoint #%d" % (waypointNum)
        marker.id = 0
        marker.type = type
        marker.action = 0
        marker.pose.position.x = 1
        marker.pose.position.y = 1
        marker.pose.position.z = 1
        marker.pose = self.pose()
        marker.scale.x = 5
        marker.scale.y = 5
        marker.scale.z = 5
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        if (type == Marker().TEXT_VIEW_FACING):
            marker.text = self.description
        return marker

    # Creates a marker for the geopoint along with a tag of the description if it exists
    def markerArray(self, frame_id, waypointNum, type):
        markerarray = MarkerArray()
        markerarray.markers.append(self.marker(frame_id, waypointNum, type, [0,1,0]))
        if len(self.description) != 0:
            markerarray.markers.append(self.marker(frame_id, 666, Marker().TEXT_VIEW_FACING, [1,1,0]))
        return markerarray

    def __str__(self):
        return '<' + str(self.lat) + ',' + str(self.lon) + " " + str(self.description) + '>'

    def __eq__(self, other):
        return self.lat == other.lat and self.lon == other.lon

    def __repr__(self):
        return "Coordinate(%d, %d) Description: %s" % (self.lat, self.lon, self.description)

# Unit tests
point1 = Geopoint(45, 73)
point2 = Geopoint(45.5, 73.5)

print("--get lat/lon point1--")
print(point1.getLat())
print(point1.getLon())
print("--get lat/lon point2--")
print(point2.getLat())
print(point2.getLon())
print("--get distance from point1 to point2")
print(point1.distanceTo(point2))
print("--get distance from point2 to point1")
print(point2.distanceTo(point1))
print("--get distanceXY from point1 to point2")
print(point1.distanceToXY(point2))
print("--get distanceXY from point2 to point1")
print(point2.distanceToXY(point1))
print("--Convert point1 deg to rad--")
print(point1.degToRad())
print("--Convert point2 deg to rad--")
print(point2.degToRad())
print("--Convert point1 rad to deg")
print(point1.degToRad().radToDeg())
print("--Convert point2 rad to deg")
print(point2.degToRad().radToDeg())


