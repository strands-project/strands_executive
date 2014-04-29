#! /usr/bin/env python

import sys
import rospy


from mdp_plan_exec.prism_client import PrismClient

from strands_executive_msgs.srv import AddMdpModel
from strands_executive_msgs.srv import GetExpectedTravelTime
from strands_executive_msgs.srv import GeneratePolicy



    
class MdpPlanner(object):

    def __init__(self):
    
        self.prism_client=PrismClient()
        self.travel_time_service = rospy.Service('/mdp_plan_exec/get_expected_travel_time', GetExpectedTravelTime, self.travel_time_cb)
        self.add_mdp_service = rospy.Service('/mdp_plan_exec/add_mdp_model', AddMdpModel, self.add_mdp_cb)
        self.generate_policy=rospy.Service('/mdp_plan_exec/generate_policy', GeneratePolicy, self.policy_cb)

        
        
        
    def travel_time_cb(self,req):
        starting_node= req.start_id
        specification='R{"time"}min=? [ (' + req.ltl_task + ') ]'
        result=self.prism_client.check_model(req.time_of_day,specification)
        result=float(result)
        return result
        
    def add_mdp_cb(self,req):
        self.prism_client.add_model(req.time_of_day,req.mdp_file)
        return True
     
    def policy_cb(self,req):
        starting_node= req.start_id
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
    

        
    mdp_planner =  MdpPlanner()
    mdp_planner.main()
