#!/usr/bin/python
# Description: Geopoint object (lat,lon) along with other useful geographic functions
#
# Authors: Jeremie Bourque
#
# Date created: 30-11-2019
# Date last updated: 30-11-2019
<<<<<<< HEAD

import rospy
import math
from geographiclib.geodesic import Geodesic
from geometry_msgs.msg import PoseStamped
=======
import math
from geographiclib.geodesic import Geodesic
>>>>>>> 8f086598513f0a85ef6771aafe080eaaecc1d31e

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
    def __init__(self, lat=200, lon=200):  # Note: 200,200 represents an invalid coordinate
        self.lat = lat
        self.lon = lon

    def getLat(self):
        return self.lat

    def getLon(self):
        return self.lon

    def setLat(self, newLat):
        self.lat = newLat

    def setLon(self, newLon):
        self.lon = newLon

    # Get a Geopoint with coordinates converted from rad to deg
    def radToDeg(self):
        geopointDeg = Geopoint()
        geopointDeg.lat = math.degrees(self.lat)
        geopointDeg.lon = math.degrees(self.lon)
        return geopointDeg

    # Get a Geopoint with coordinates converted from rad to deg
    # Slightly modified from http://wiki.ros.org/gps_goal
    def degToRad(self):
        geopointRad = Geopoint()
        geopointRad.lat = math.radians(self.lat)
        geopointRad.lon = math.radians(self.lon)
        return geopointRad

    # Returns distance in meters from self to a target point
    def distanceTo(self, target):
        # Calculate distance and azimuth between GPS points
        geod = Geodesic.WGS84  # define the WGS84 ellipsoid
        g = geod.Inverse(self.lat, self.lon, target.lat,
                         target.lon)  # Compute several geodesic calculations between two GPS points
        hypotenuse = distance = g['s12']  # access distance
        return hypotenuse

    # Returns distance in meters from self to a target point in the X and Y directions
    # Slightly modified from http://wiki.ros.org/gps_goal
    def distanceToXY(self, target):
        # Calculate distance and azimuth between GPS points
        geod = Geodesic.WGS84  # define the WGS84 ellipsoid
        g = geod.Inverse(self.lat, self.lon, target.lat,
                         target.lon)  # Compute several geodesic calculations between two GPS points
        hypotenuse = distance = g['s12']  # access distance
        azimuth = g['azi1']

        # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) by finding side lengths of a right-angle triangle
        # Convert azimuth to radians
        azimuth = math.radians(azimuth)
        x = adjacent = math.cos(azimuth) * hypotenuse
        y = opposite = math.sin(azimuth) * hypotenuse

        return x, y

<<<<<<< HEAD
    # Converts geopoint to a PoseStamped message
    def poseStamped(self):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "wgs84"
        pose.pose.position.x = self.lon
        pose.pose.position.y = self.lat
        return pose

=======
>>>>>>> 8f086598513f0a85ef6771aafe080eaaecc1d31e
    def __str__(self):
        return '<' + str(self.lat) + ',' + str(self.lon) + '>'

    def __eq__(self, other):
        return self.lat == other.lat and self.lon == other.lon

    def __repr__(self):
        return "Coordinate(%d, %d)" % (self.lat, self.lon)

# Unit tests
# point1 = Geopoint(45, 73)
# point2 = Geopoint(45.5, 73.5)
#
# print("--get lat/lon point1--")
# print(point1.getLat())
# print(point1.getLon())
# print("--get lat/lon point2--")
# print(point2.getLat())
# print(point2.getLon())
# print("--get distance from point1 to point2")
# print(point1.distanceTo(point2))
# print("--get distance from point2 to point1")
# print(point2.distanceTo(point1))
# print("--get distanceXY from point1 to point2")
# print(point1.distanceToXY(point2))
# print("--get distanceXY from point2 to point1")
# print(point2.distanceToXY(point1))
# print("--Convert point1 deg to rad--")
# print(point1.degToRad())
# print("--Convert point2 deg to rad--")
# print(point2.degToRad())
# print("--Convert point1 rad to deg")
# print(point1.degToRad().radToDeg())
# print("--Convert point2 rad to deg")
# print(point2.degToRad().radToDeg())

