#!/usr/bin/python

import math
import rospy

from Swarm_Drone_API import MAVROS_Drone
from SwarmFollower_API import SwarmFollower


class SwarmLeader(MAVROS_Drone):
    def __init__(self, name='drone1', n_followers=2, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        self.ns = name
        self.data.header.name = name
        self.data.header.id = 1
        
        self.n_followers = n_followers
        self.followers = []
        
        # Create SwarmFollower instances and store them in self.followers list
        for i in range(n_followers):
            this_follower = SwarmFollower()
            this_follower.data.header.name = 'drone'+str(i+2)
            this_follower.ns = 'drone'+str(i+2)
            this_follower.data.header.id = i+2
            self.followers.append(this_follower)
        
        rospy.init_node('Swarm_Leader')
        
        self.initialize()
        
        # Wait for GPS Fix
        self.wait_for_GPS_Fix()  
        
    def initialize(self):
        # Initialize leader's ROS subscribers and publishers
        self.init_subscribers()
        self.init_publishers()
        
        # Initialize follower's ROS subscribers and publishers
        for i in range(self.n_followers):
            self.followers[i].init_subscribers()
            self.followers[i].init_publishers()
            
    def wait_for_GPS_Fix(self):
        while True:
            if self.data.global_position.gps_fix == 0:
                print("Leader is ready!")
                if self.check_followers_GPS_Fix():
                    print("All followers are ready!")
                    break
        
    def check_followers_GPS_Fix(self):
        followers_ready = 0    
        for follower in self.followers:
            if follower.data.global_position.gps_fix == 0:
                followers_ready += 1
        
        if followers_ready == self.n_followers:
            return True
        else:
            return False
        
    def calculate_follower_coordinates(self):
        
        # Earth radius in meters
        EARTH_RADIUS = 6378137.0  # Approximate value for WGS84 ellipsoid
        
        follower_coordinates = []

        # Convert latitude and longitude from degrees to radians
        leader_latitude_rad = math.radians(self.data.global_position.latitude)
        leader_longitude_rad = math.radians(self.data.global_position.longitude)

        # Convert heading from degrees to radians
        heading_rad = math.radians(self.data.euler_orientation.yaw)

        # Calculate initial change in latitude and longitude (2 meters behind the leader)
        delta_latitude = 2 / EARTH_RADIUS
        delta_longitude = 2 / (EARTH_RADIUS * math.cos(leader_latitude_rad))

        for i in range(self.n_followers):
            # Calculate new latitude and longitude for each follower
            new_latitude_rad = leader_latitude_rad - ((i+1) * delta_latitude)
            new_longitude_rad = leader_longitude_rad - ((i+1) * delta_longitude)

            # Convert new latitude and longitude from radians to degrees
            new_latitude = math.degrees(new_latitude_rad)
            new_longitude = math.degrees(new_longitude_rad)

            # Add the coordinates to the list
            follower_coordinates.append((new_latitude, new_longitude))

        return follower_coordinates
                 
    def leader_GPS_Subscriber_callback(self, mssg):
        self.GPS_Fix = mssg.status.status

        self.leader_data.global_position.latitude = mssg.latitude
        self.leader_data.global_position.longitude = mssg.longitude
        self.leader_data.global_position.altitude = mssg.altitude
        
    def leader_LocalPos_Subscriber_callback(self, mssg):
        self.leader_data.local_position.x = mssg.pose.pose.position.x
        self.leader_data.local_position.y = mssg.pose.pose.position.y
        self.leader_data.local_position.z = mssg.pose.pose.position.z
        
    def leader_Heading_Subscriber_callback(self,mssg):
        self.leader_data.euler_orientation.yaw = mssg.data
        
# """
# Functions to make it easy to convert between the different frames-of-reference. In particular these
# make it easy to navigate in terms of "metres from the current position" when using commands that take 
# absolute positions in decimal degrees.

# The methods are approximations only, and may be less accurate over longer distances, and when close 
# to the Earth's poles.

# Specifically, it provides:
# * get_location_metres - Get LocationGlobal (decimal degrees) at distance (m) North & East of a given LocationGlobal.
# * get_distance_metres - Get the distance between two LocationGlobal objects in metres
# * get_bearing - Get the bearing in degrees to a LocationGlobal
# """

# def get_location_metres(original_location, dNorth, dEast):
#     """
#     Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
#     specified `original_location`. The returned LocationGlobal has the same `alt` value
#     as `original_location`.

#     The function is useful when you want to move the vehicle around specifying locations relative to 
#     the current vehicle position.

#     The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

#     For more information see:
#     http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
#     """
#     earth_radius = 6378137.0 #Radius of "spherical" earth
#     #Coordinate offsets in radians
#     dLat = dNorth/earth_radius
#     dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

#     #New position in decimal degrees
#     newlat = original_location.lat + (dLat * 180/math.pi)
#     newlon = original_location.lon + (dLon * 180/math.pi)
#     if type(original_location) is LocationGlobal:
#         targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
#     elif type(original_location) is LocationGlobalRelative:
#         targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
#     else:
#         raise Exception("Invalid Location object passed")
        
#     return targetlocation;


# def get_distance_metres(aLocation1, aLocation2):
#     """
#     Returns the ground distance in metres between two LocationGlobal objects.

#     This method is an approximation, and will not be accurate over large distances and close to the 
#     earth's poles. It comes from the ArduPilot test code: 
#     https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
#     """
#     dlat = aLocation2.lat - aLocation1.lat
#     dlong = aLocation2.lon - aLocation1.lon
#     return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


# def get_bearing(aLocation1, aLocation2):
#     """
#     Returns the bearing between the two LocationGlobal objects passed as parameters.

#     This method is an approximation, and may not be accurate over large distances and close to the 
#     earth's poles. It comes from the ArduPilot test code: 
#     https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
#     """	
#     off_x = aLocation2.lon - aLocation1.lon
#     off_y = aLocation2.lat - aLocation1.lat
#     bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
#     if bearing < 0:
#         bearing += 360.00
#     return bearing;