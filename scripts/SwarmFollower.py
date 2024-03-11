#!/usr/bin/python

from MAVROS_Drone import MAVROS_Drone

class SwarmFollower(MAVROS_Drone):
    def __init__(self) -> None:
        pass
        
        
        
if __name__ == '__main__':
    
    leader = SwarmFollower()