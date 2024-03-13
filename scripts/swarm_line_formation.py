#!/usr/bin/python

import time
import sys

from SwarmLeader_API import *

leader = SwarmLeader(name='drone1', n_followers=2)

rospy.loginfo("Setting Leader's Mode to GUIDED")
leader_response = leader.set_mode(mode='GUIDED')

if leader_response.mode_sent:
    rospy.loginfo("Leader's Mode is now GUIDED.")
else:
    rospy.logerr("Leader's Mode cannot be switched to GUIDED. Aborting Mission...")
    sys.exit(1)

rospy.loginfo("Setting Followers Mode to GUIDED")
for follower in leader.followers:
    follower_response = follower.set_mode(mode='GUIDED')
    
    if follower_response.mode_sent:
        rospy.loginfo("{} has been switched to GUIDED.".format(follower.data.header.name))
    else:
        rospy.loginfo("{} cannot be switched to GUIDED.".format(follower.data.header.name))
        
        rospy.loginfo("Removing {} from list of active followers.".format(follower.data.header.name))
        leader.followers.remove(follower)
        leader.n_followers = leader.n_followers - 1
    
rospy.loginfo("Arming Leader")
leader_response = leader.arm()

if leader_response.success:
    rospy.loginfo("Leader's Mode is now ARMED.")
else:
    rospy.logerr("Leader is not Armable. Aborting Mission...")
    sys.exit(1)
    
# Leader Take-off
target_altitude = (2*leader.n_followers)+2          # 2m spacing between each drone
leader_response = leader.takeoff(altitude=target_altitude)

if leader_response.success:
    rospy.loginfo("Leader is Taking off.")
    while not leader.check_takeoff_complete():
        # Waiting for leader to complete takeoff
        time.sleep(0.1)
else:
    rospy.logerr("Leader failed to takeoff. Aborting Mission...")
    sys.exit(1)
    
rospy.loginfo("Getting Follower Coordinates")
follower_coordinates = leader.calculate_follower_coordinates()

rospy.loginfo("Arming Followers")
for follower in leader.followers:
    follower_response = follower.arm()
    
    if follower_response.success:
        rospy.loginfo("{} is ARMED.".format(follower.data.header.name))
    else:
        rospy.loginfo("{} cannot be ARMED.".format(follower.data.header.name))
        
        # rospy.loginfo("Removing {follower.data.header.name} from list of active followers.")
        # leader.followers.remove(follower)
        # leader.n_followers = leader.n_followers - 1

# Followers Take-off
for follower in leader.followers:
    follower_index = leader.followers.index(follower)
    follower_response = follower.takeoff(altitude=(2*(leader.n_followers-(follower_index+1)))+2)
    
    if follower_response.success:
        rospy.loginfo("{} is Taking off.".format(follower.data.header.name))
        while not follower.check_takeoff_complete():
            # Waiting for follower to complete takeoff
            time.sleep(0.1)
    else:
        rospy.loginfo("{} failed to takeoff.".format(follower.data.header.name))
    
        # rospy.loginfo("Removing {follower.data.header.name} from list of active followers.")
        # leader.followers.remove(follower)
        # leader.n_followers = leader.n_followers - 1
        
heading = leader.data.euler_orientation.yaw
        
# Line Formation
for follower in leader.followers:
    follower_index = leader.followers.index(follower)
    rospy.loginfo("Setting RTL_ALT param of {}.".format(follower.data.header.name))
    follower.set_param(param_name="RTL_ALT", param_value=int((2*(leader.n_followers-(follower_index+1)))+2))
    # follower.goto_location(latitude=follower_coordinates[follower_index][0], longitude=follower_coordinates[follower_index][1], altitude=(2*(leader.n_followers-(follower_index+1)))+2)
    follower.goto_location_heading(latitude=follower_coordinates[follower_index][0], 
                                   longitude=follower_coordinates[follower_index][1], 
                                   altitude=(2*(leader.n_followers-(follower_index+1)))+2,
                                   yaw=heading)
    time.sleep(2)
    
# Hover for few seconds
rospy.loginfo("Takeoff Complete. Hovering...")
time.sleep(5)

# Land all drones
rospy.loginfo("Landing followers...")
for follower in leader.followers:
    # follower_response = follower.set_mode(mode='LAND')
    follower_response = follower.set_mode(mode='RTL')
    
rospy.loginfo("Landing Leader...")
leader.set_mode(mode="LAND")
    
for follower in leader.followers:
    while not follower.check_land_complete():
        # print("...")
        time.sleep(0.1)
rospy.loginfo("All Followers have landed.")

while not leader.check_land_complete():
    # print("...")
    time.sleep(0.1)
    
time.sleep(2)
    
rospy.loginfo("Leader has successfully landed.")
    
rospy.loginfo("Mission Successfull.")