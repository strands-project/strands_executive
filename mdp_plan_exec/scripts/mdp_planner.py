#! /usr/bin/env python

import sys
import rospy
import os


from mdp_plan_exec.prism_client import PrismClient
from mdp_plan_exec.mdp import TopMapMdp

from strands_executive_msgs.srv import AddMdpModel
from strands_executive_msgs.srv import GetExpectedTravelTime
from strands_executive_msgs.srv import GeneratePolicy



    
class MdpPlanner(object):

    def __init__(self,top_map):
    
        self.prism_client=PrismClient()
        self.travel_time_service = rospy.Service('/mdp_plan_exec/get_expected_travel_time', GetExpectedTravelTime, self.travel_time_cb)
        self.add_mdp_service = rospy.Service('/mdp_plan_exec/add_mdp_model', AddMdpModel, self.add_mdp_cb)
        self.generate_policy=rospy.Service('/mdp_plan_exec/generate_policy', GeneratePolicy, self.policy_cb)

        self.top_map_mdp=TopMapMdp(top_map)
        self.top_map_mdp.update_nav_statistics()
        
        self.directory = os.path.expanduser("~") + '/tmp/prism'
        try:
            os.makedirs(self.directory)
        except OSError as ex:
            print 'error creating PRISM directory:',  ex
            
        self.mdp_prism_file=self.directory+'/'+top_map+'.prism'    
        
        self.top_map_mdp.write_prism_model(self.mdp_prism_file)
        
        self.prism_client.add_model('all_day',self.mdp_prism_file)
        
        
    
        
    def travel_time_cb(self,req):
        starting_node= req.start_id
        self.top_map_mdp.set_initial_state_from_name(starting_node)
        self.top_map_mdp.write_prism_model(self.mdp_prism_file)
        result=self.prism_client.update_model(req.time_of_day,self.mdp_prism_file)
        specification='R{"time"}min=? [ (' + req.ltl_task + ') ]'
        result=self.prism_client.check_model(req.time_of_day,specification)
        result=float(result)
        return result
        
    def add_mdp_cb(self,req):
        self.prism_client.add_model(req.time_of_day,req.mdp_file)
        return True
     
    def policy_cb(self,req):
        starting_node= req.start_id
        self.top_map_mdp.set_initial_state_from_name(starting_node)
        self.top_map_mdp.write_prism_model(self.mdp_prism_file)
        result=self.prism_client.update_model(self.mdp_prism_file)
        specification='R{"time"}min=? [ (' + req.ltl_task + ') ]'
        result=self.prism_client.get_policy(req.time_of_day,specification)
        return True
        
        
        
        
        
    
    def main(self):


        # Wait for control-c
        rospy.spin()
        
        if rospy.is_shutdown():
            self.prism_client.shutdown(True)


if __name__ == '__main__':
    rospy.init_node('mdp_planner')
    
    if len(sys.argv)<2:
        print "usage: rosrun mdp_plan_exec mdp_planner <topological_map_name>"
        sys.exit(2)
        
    mdp_planner =  MdpPlanner(sys.argv[1])
    
    
    
    
    mdp_planner.main()
    
    
