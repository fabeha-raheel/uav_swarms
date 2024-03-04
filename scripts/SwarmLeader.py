#!/usr/bin/python

import time
import rospy

from sensor_msgs.msg import NavSatFix
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
        
        rospy.init_node('Swarm_Leader')
        
        # Initialize Leader and Followers subscribers
        self.init_subscribers()
        
        # Wait for GPS Fix
        self.wait_for_GPS_Fix()            
        
    def init_subscribers(self):
        # create a global_position_subcriber for the leader
        self.leader_global_position_subscriber = rospy.Subscriber(self.leader_data.header.name + '/mavros/global_position/global',NavSatFix, self.leader_GPS_Subscriber_callback)
    
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

    def disarm(self):
        rospy.wait_for_service(self.leader_data.header.name + '/mavros/cmd/arming', timeout=3)
        try:
            armService = rospy.ServiceProxy(self.leader_data.header.name + '/mavros/cmd/arming', CommandBool)
            armResponse = armService(False)
            rospy.loginfo(armResponse)
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)
            
            
    def set_mode(self, mode):
        rospy.wait_for_service(self.leader_data.header.name + '/mavros/set_mode', timeout=3)
        try:
            change_mode = rospy.ServiceProxy(self.leader_data.header.name + '/mavros/set_mode', SetMode)
            response = change_mode(custom_mode=mode)
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            print("Set mode failed: %s" %e)
            
    def takeoff(self, altitude=5, latitude=0, longitude=0, min_pitch=0, yaw=0):
        rospy.wait_for_service(self.leader_data.header.name + '/mavros/cmd/takeoff', timeout=3)
        try:
            takeoff_service = rospy.ServiceProxy(self.leader_data.header.name + '/mavros/cmd/takeoff', CommandTOL)
            response = takeoff_service(altitude=altitude, latitude=latitude, longitude=longitude, min_pitch=min_pitch, yaw=yaw)
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            print("Takeoff failed: %s" %e)     
    
    def leader_GPS_Subscriber_callback(self, mssg):
        self.GPS_Fix = mssg.status.status

        self.leader_data.global_position.latitude = mssg.latitude
        self.leader_data.global_position.longitude = mssg.longitude
        self.leader_data.global_position.altitude = mssg.altitude
        
    def swarm_arm_and_takeoff(self):
        print("Setting Leader's Mode to GUIDED")
        self.set_mode(mode='GUIDED')
        
        print("Setting Followers Mode to GUIDED")
        for follower in self.followers:
            follower.set_mode(mode='GUIDED')
        
        print("Arming Leader")
        self.arm()
        
        print("Arming Followers")
        for follower in self.followers:
            follower.arm()
            
        print("Leader taking off...")
        self.takeoff(altitude=(2*self.n_followers)+2)
        
        print("Followers taking off...")
        for follower in self.followers:
            follower_index = self.followers.index(follower)
            follower.takeoff(altitude=(2*(self.n_followers-(follower_index+1)))+2)
            
        # Wait for few seconds
        time.sleep(10)
        
        # Land all drones
        print("Landing followers...")
        for follower in self.followers:
            follower.set_mode(mode='LAND')
            
        time.sleep(2)
        
        print("Landing Leader...")
        self.set_mode(mode="LAND")
        
