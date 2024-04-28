#!/usr/bin/python

from Swarm_Drone_API import MAVROS_Drone
#from SwarmLeader_API import SwarmLeader

class SwarmFollower(MAVROS_Drone):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        pass
    
    #def register_leader(self, leader_name):
    #    self.leader = SwarmLeader(name=leader_name, n_followers=2)
        
    
        
        
if __name__ == '__main__':
    
    leader = SwarmFollower()