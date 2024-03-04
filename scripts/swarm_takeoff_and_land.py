#!/usr/bin/python

from SwarmLeader import *

leader = SwarmLeader(name='drone1', n_followers=2)

leader.swarm_arm_and_takeoff()