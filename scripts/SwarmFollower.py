#!/usr/bin/python

import rospy

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

from Drone_Data import Drone_Data

class SwarmFollower():
    def __init__(self) -> None:

        self.data = Drone_Data()
        self.GPS_Fix = None
        self.takeoff_altitude = 0
        
    def init_subscribers(self):
        # create a global_position_subcriber for the follower
        self.follower_global_position_subscriber = rospy.Subscriber(self.data.header.name + '/mavros/global_position/global',NavSatFix, self.follower_GPS_Subscriber_callback)
        self.follower_local_position_subscriber = rospy.Subscriber(self.data.header.name + '/mavros/global_position/local',Odometry, self.follower_LocalPos_Subscriber_callback)
        
    def arm(self):
        rospy.wait_for_service(self.data.header.name + '/mavros/cmd/arming', timeout=3)
        try:
            armService = rospy.ServiceProxy(self.data.header.name + '/mavros/cmd/arming', CommandBool)
            armResponse = armService(True)
            rospy.loginfo(armResponse)
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)
            
        return armResponse

    def disarm(self):
        rospy.wait_for_service(self.data.header.name + '/mavros/cmd/arming', timeout=3)
        try:
            armService = rospy.ServiceProxy(self.data.header.name + '/mavros/cmd/arming', CommandBool)
            armResponse = armService(False)
            rospy.loginfo(armResponse)
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e) 
            
        return armResponse
            
    def set_mode(self, mode):
        rospy.wait_for_service(self.data.header.name + '/mavros/set_mode', timeout=3)
        try:
            change_mode = rospy.ServiceProxy(self.data.header.name + '/mavros/set_mode', SetMode)
            response = change_mode(custom_mode=mode)
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            print("Set mode failed: %s" %e) 
            
        return response
            
    def takeoff(self, altitude=5, latitude=0, longitude=0, min_pitch=0, yaw=0):
        rospy.wait_for_service(self.data.header.name + '/mavros/cmd/takeoff', timeout=3)
        try:
            takeoff_service = rospy.ServiceProxy(self.data.header.name + '/mavros/cmd/takeoff', CommandTOL)
            response = takeoff_service(altitude=altitude, latitude=latitude, longitude=longitude, min_pitch=min_pitch, yaw=yaw)
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            print("Takeoff failed: %s" %e)
            
        self.takeoff_altitude = altitude
            
        return response
    
    def check_takeoff_complete(self):
        error = abs(self.takeoff_altitude - self.data.local_position.z)
        print("Takeoff error: ", error)
        if error <= 0.2:
            return True
        else:
            return False
        
    def check_land_complete(self):
        if self.data.local_position.z <= 0.5:
            return True
        else:
            return False
    
    def follower_GPS_Subscriber_callback(self, mssg):
        
        self.GPS_Fix = mssg.status.status
        
        self.data.global_position.latitude = mssg.latitude
        self.data.global_position.longitude = mssg.longitude
        self.data.global_position.altitude = mssg.altitude
        
    def follower_LocalPos_Subscriber_callback(self, mssg):
        self.data.local_position.x = mssg.pose.pose.position.x
        self.data.local_position.y = mssg.pose.pose.position.y
        self.data.local_position.z = mssg.pose.pose.position.z
        
        
if __name__ == '__main__':
    
    leader = SwarmFollower()