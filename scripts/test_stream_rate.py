#!/usr/bin/python3

import time

from SwarmLeader_API import *

leader = SwarmLeader(name='drone1', n_followers=2)

rospy.loginfo("Setting Stream Rate")
leader.set_stream_rate()
time.sleep(0.5)