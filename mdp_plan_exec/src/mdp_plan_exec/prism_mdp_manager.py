#!/usr/bin/python

import os
import sys
import rospy


from  mdp import TopMapMdp, ProductMdp
from prism_client import PrismClient

class PrismMdpManager(object):
    
    def __init__(self,port, work_dir, top_map):
        self.directory = os.path.expanduser("~") + '/tmp/prism/' + work_dir + "/"
        try:
            os.makedirs(self.directory)
        except OSError as ex:
            print 'error creating PRISM directory:',  ex
            
        self.prism_client=PrismClient(port, self.directory)
        self.top_map_mdp=TopMapMdp(top_map)
        self.top_map_mdp.update_nav_statistics()
                
        self.mdp_prism_file=self.directory + top_map + '.prism'    
        
        self.top_map_mdp.write_prism_model(self.mdp_prism_file)
        
        self.prism_client.add_model('all_day',self.mdp_prism_file)
        
        self.product_mdp = None
        
    def update_current_top_mdp(self,time_of_day,update_stats=True):
        if update_stats:
            self.top_map_mdp.update_nav_statistics()
        self.top_map_mdp.write_prism_model(self.mdp_prism_file)
        result=self.prism_client.update_model(time_of_day,self.mdp_prism_file)
        
        
    def get_working_dir(self):
        return self.directory