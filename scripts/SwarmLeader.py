#!/usr/bin/python

import rospy

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

from Drone_Data import Drone_Data
from SwarmFollower import SwarmFollower


class SwarmLeader():
    def __init__(self, name='drone1', n_followers=2) -> None:

        self.leader_data = Drone_Data()
        
        self.leader_data.header.name = name
        self.leader_data.header.id = 1
        
        self.n_followers = n_followers
        self.followers = []
        
        # Create SwarmFollower instances and store them in self.followers list
        for i in range(n_followers):
            this_follower = SwarmFollower()
            this_follower.data.header.name = 'drone'+str(i+2)
            this_follower.data.header.id = i+2
            self.followers.append(this_follower)
        
        self.GPS_Fix = None
        
        self.takeoff_altitude = 0
        
        rospy.init_node('Swarm_Leader')
        
        # Initialize Leader and Followers subscribers
        self.init_subscribers()
        
        # Wait for GPS Fix
        self.wait_for_GPS_Fix()            
        
    def init_subscribers(self):
        # create a global_position_subcriber for the leader
        self.leader_global_position_subscriber = rospy.Subscriber(self.leader_data.header.name + '/mavros/global_position/global',NavSatFix, self.leader_GPS_Subscriber_callback)
        self.leader_local_position_subscriber = rospy.Subscriber(self.leader_data.header.name + '/mavros/global_position/local',Odometry, self.leader_LocalPos_Subscriber_callback)
    
        # initialize the global position subscribers each of the followers
        for i in range(self.n_followers):
            self.followers[i].init_subscribers()
            
    def wait_for_GPS_Fix(self):
        while True:
            if self.GPS_Fix == 0:
                print("Leader is ready!")
                followers_ready = self.check_followers_GPS_Fix()
                if followers_ready == self.n_followers:
                    print("All followers are ready!")
                    break
        
    def check_followers_GPS_Fix(self):
        followers_ready = 0    
        for follower in self.followers:
            if follower.GPS_Fix == 0:
                followers_ready += 1
        
        return followers_ready
    
    def arm(self):
        rospy.wait_for_service(self.leader_data.header.name + '/mavros/cmd/arming', timeout=3)
        try:
            armService = rospy.ServiceProxy(self.leader_data.header.name + '/mavros/cmd/arming', CommandBool)
            armResponse = armService(True)
            rospy.loginfo(armResponse)
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)
            
        return armResponse

    def disarm(self):
        rospy.wait_for_service(self.leader_data.header.name + '/mavros/cmd/arming', timeout=3)
        try:
            armService = rospy.ServiceProxy(self.leader_data.header.name + '/mavros/cmd/arming', CommandBool)
            armResponse = armService(False)
            rospy.loginfo(armResponse)
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)
            
        return armResponse
            
            
    def set_mode(self, mode):
        rospy.wait_for_service(self.leader_data.header.name + '/mavros/set_mode', timeout=3)
        try:
            change_mode = rospy.ServiceProxy(self.leader_data.header.name + '/mavros/set_mode', SetMode)
            response = change_mode(custom_mode=mode)
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            print("Set mode failed: %s" %e)
            
        return response
            
    def takeoff(self, altitude=5, latitude=0, longitude=0, min_pitch=0, yaw=0):
        rospy.wait_for_service(self.leader_data.header.name + '/mavros/cmd/takeoff', timeout=3)
        try:
            takeoff_service = rospy.ServiceProxy(self.leader_data.header.name + '/mavros/cmd/takeoff', CommandTOL)
            response = takeoff_service(altitude=altitude, latitude=latitude, longitude=longitude, min_pitch=min_pitch, yaw=yaw)
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            print("Takeoff failed: %s" %e)
            
        self.takeoff_altitude = altitude
            
        return response
    
    def check_takeoff_complete(self):
        error = abs(self.takeoff_altitude - self.leader_data.local_position.z)
        print("Takeoff error: ", error)
        if error <= 0.2:
            return True
        else:
            return False
        
    def check_land_complete(self):
        if self.leader_data.local_position.z <= 0.5:
            return True
        else:
            return False
                 
    
    def leader_GPS_Subscriber_callback(self, mssg):
        self.GPS_Fix = mssg.status.status

        self.leader_data.global_position.latitude = mssg.latitude
        self.leader_data.global_position.longitude = mssg.longitude
        self.leader_data.global_position.altitude = mssg.altitude
        
    def leader_LocalPos_Subscriber_callback(self, mssg):
        self.leader_data.local_position.x = mssg.pose.pose.position.x
        self.leader_data.local_position.y = mssg.pose.pose.position.y
        self.leader_data.local_position.z = mssg.pose.pose.position.z
        
