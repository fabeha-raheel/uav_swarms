#!/usr/bin/python

from SwarmLeader import *

leader = SwarmLeader(name='drone1', n_followers=2)

# leader.swarm_arm_and_takeoff()

print("Setting Leader's Mode to GUIDED")
leader.set_mode(mode='GUIDED')

print("Setting Followers Mode to GUIDED")
for follower in leader.followers:
    follower.set_mode(mode='GUIDED')
    
print("Arming Leader")
leader.arm()

print("Arming Followers")
for follower in leader.followers:
    follower.arm()
    
print("Leader taking off...")
leader.takeoff(altitude=(2*leader.n_followers)+2)

print("Followers taking off...")
for follower in leader.followers:
    follower_index = leader.followers.index(follower)
    follower.takeoff(altitude=(2*(leader.n_followers-(follower_index+1)))+2)
    
# Wait for few seconds
time.sleep(10)

# Land all drones
print("Landing followers...")
for follower in leader.followers:
    follower.set_mode(mode='LAND')
    
time.sleep(2)

print("Landing Leader...")
leader.set_mode(mode="LAND")