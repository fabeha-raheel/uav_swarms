#!/usr/bin/python

import math
import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from mavros_msgs.msg import GlobalPositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

from Drone_Data import Drone_Data


class MAVROS_Drone():
    def __init__(self, ns=None) -> None:

        self.ns = ns
        self.data = Drone_Data()
        
    def init_node(self):
        rospy.init_node("MAVROS_Drone_Node")           
        
    def init_subscribers(self):
        if self.ns is not None:
            self.global_position_subscriber = rospy.Subscriber(self.ns + '/mavros/global_position/global',NavSatFix, self.global_sub_cb)
            self.local_position_subscriber = rospy.Subscriber(self.ns + '/mavros/global_position/local',Odometry, self.local_sub_cb)
            self.compass_hdg_subscriber = rospy.Subscriber(self.ns + '/mavros/global_position/compass_hdg',Float64, self.hdg_sub_cb)        
        else:
            self.global_position_subscriber = rospy.Subscriber('/mavros/global_position/global',NavSatFix, self.global_sub_cb)
            self.local_position_subscriber = rospy.Subscriber('/mavros/global_position/local',Odometry, self.local_sub_cb)
            self.compass_hdg_subscriber = rospy.Subscriber('/mavros/global_position/compass_hdg',Float64, self.hdg_sub_cb)
            
    def init_publishers(self):
        if self.ns is not None:
            self.global_setpoint_publisher = rospy.Publisher(self.ns + '/mavros/setpoint_raw/global',GlobalPositionTarget, queue_size=1)
        else:
            self.global_setpoint_publisher = rospy.Publisher('/mavros/setpoint_raw/global',GlobalPositionTarget, queue_size=1)
            
    def check_GPS_fix(self):
        if self.data.global_position.gps_fix >= 0:
            return True
        else:
            return False
    
    def arm(self):
        if self.ns is not None:
            rospy.wait_for_service(self.ns + '/mavros/cmd/arming', timeout=3)
            try:
                armService = rospy.ServiceProxy(self.ns + '/mavros/cmd/arming', CommandBool)
                armResponse = armService(True)
                rospy.loginfo(armResponse)
            except rospy.ServiceException as e:
                print("Service call failed: %s" %e)
        else:
            rospy.wait_for_service('/mavros/cmd/arming', timeout=3)
            try:
                armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
                armResponse = armService(True)
                rospy.loginfo(armResponse)
            except rospy.ServiceException as e:
                print("Service call failed: %s" %e)
        
        return armResponse

    def disarm(self):
        if self.ns is not None:
            rospy.wait_for_service(self.ns + '/mavros/cmd/arming', timeout=3)
            try:
                armService = rospy.ServiceProxy(self.ns + '/mavros/cmd/arming', CommandBool)
                armResponse = armService(False)
                rospy.loginfo(armResponse)
            except rospy.ServiceException as e:
                print("Service call failed: %s" %e)
        else:
            rospy.wait_for_service('/mavros/cmd/arming', timeout=3)
            try:
                armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
                armResponse = armService(False)
                rospy.loginfo(armResponse)
            except rospy.ServiceException as e:
                print("Service call failed: %s" %e)
        
        return armResponse
            
            
    def set_mode(self, mode):
        if self.ns is not None:
            rospy.wait_for_service(self.ns + '/mavros/set_mode', timeout=3)
            try:
                modeService = rospy.ServiceProxy(self.ns + '/mavros/set_mode', SetMode)
                modeResponse = modeService(custom_mode=mode)
                rospy.loginfo(modeResponse)
            except rospy.ServiceException as e:
                print("Set mode failed: %s" %e)
        else:
            rospy.wait_for_service('/mavros/set_mode', timeout=3)
            try:
                modeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
                modeResponse = modeService(custom_mode=mode)
                rospy.loginfo(modeResponse)
            except rospy.ServiceException as e:
                print("Set mode failed: %s" %e)
        
        return modeResponse
            
    def takeoff(self, altitude=5, latitude=0, longitude=0, min_pitch=0, yaw=0):
        if self.ns is None:
            rospy.wait_for_service('/mavros/cmd/takeoff', timeout=3)
            try:
                takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
                takeoffResponse = takeoffService(altitude=altitude, latitude=latitude, longitude=longitude, min_pitch=min_pitch, yaw=yaw)
                rospy.loginfo(takeoffResponse)
            except rospy.ServiceException as e:
                print("Takeoff failed: %s" %e)
        else:
            rospy.wait_for_service(self.ns + '/mavros/cmd/takeoff', timeout=3)
            try:
                takeoffService = rospy.ServiceProxy(self.ns + '/mavros/cmd/takeoff', CommandTOL)
                takeoffResponse = takeoffService(altitude=altitude, latitude=latitude, longitude=longitude, min_pitch=min_pitch, yaw=yaw)
                rospy.loginfo(takeoffResponse)
            except rospy.ServiceException as e:
                print("Takeoff failed: %s" %e)
            
        if takeoffResponse:
            self.takeoff_altitude = altitude
        else:
            self.takeoff_altitude = 0
            
        return takeoffResponse
    
    def goto_location(self, latitude, longitude, altitude, type_mask=4088, coordinate_frame=6):
        
        command = GlobalPositionTarget()
        command.altitude = altitude
        command.latitude = latitude
        command.longitude = longitude
        command.type_mask=type_mask
        command.coordinate_frame=coordinate_frame
        
        self.global_setpoint_publisher.publish(command)
        
    def goto_location_heading(self, latitude, longitude, altitude, yaw, type_mask=4088, coordinate_frame=6):
        
        command = GlobalPositionTarget()
        command.altitude = altitude
        command.latitude = latitude
        command.longitude = longitude
        command.yaw = yaw
        command.type_mask=GlobalPositionTarget.IGNORE_VX | GlobalPositionTarget.IGNORE_VY | GlobalPositionTarget.IGNORE_VZ | GlobalPositionTarget.IGNORE_AFX | GlobalPositionTarget.IGNORE_AFY | GlobalPositionTarget.IGNORE_AFZ | GlobalPositionTarget.IGNORE_YAW_RATE
        command.coordinate_frame=coordinate_frame
        
        self.global_setpoint_publisher.publish(command)
    
    def check_takeoff_complete(self):
        if abs(self.takeoff_altitude - self.data.local_position.z) <= 0.2:
            return True
        else:
            return False
        
    def check_land_complete(self):
        if self.data.local_position.z <= 0.5:
            return True
        else:
            return False
        
    def check_target_location_reached(self, location):
        pass
                 
    def global_sub_cb(self, mssg):
        self.data.global_position.gps_fix = mssg.status.status
        self.data.global_position.latitude = mssg.latitude
        self.data.global_position.longitude = mssg.longitude
        self.data.global_position.altitude = mssg.altitude
        
    def local_sub_cb(self, mssg):
        self.data.local_position.x = mssg.pose.pose.position.x
        self.data.local_position.y = mssg.pose.pose.position.y
        self.data.local_position.z = mssg.pose.pose.position.z
        
    def hdg_sub_cb(self,mssg):
        self.data.euler_orientation.yaw = mssg.data
        
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