#! /usr/bin/env python

import sys
import rospy
import rospkg


from strands_executive_msgs.srv import GetExpectedTravelTime, GetExpectedTravelTimeRequest
from strands_executive_msgs.srv import AddMdpModel, AddMdpModelRequest
from strands_executive_msgs.srv import GeneratePolicy, GeneratePolicyRequest


from mdp_plan_exec.mdp import TopMapMdp, ProductMdp





    
class MdpPlanner(object):

    def __init__(self):
    
        # Create the main state machine
        self.time_client= rospy.ServiceProxy('/mdp_plan_exec/get_expected_travel_time', GetExpectedTravelTime)
        self.add_client= rospy.ServiceProxy('/mdp_plan_exec/add_mdp_model', AddMdpModel)
        self.policy_client= rospy.ServiceProxy('/mdp_plan_exec/generate_policy', GeneratePolicy)

        
        
        

    
    
    def main(self):
        
        
        
        top_map_mdp=TopMapMdp('top_floor_simple')
        top_map_mdp.update_nav_statistics()

    
        #top_map_mdp.set_policy('/home/bruno/tmp/prism/all_day/adv.tra')
        top_map_mdp.write_prism_model('/home/bruno/Desktop/teste.prism')
        
        req=AddMdpModelRequest()
        req.time_of_day='all_day'
        req.mdp_file='/home/bruno/Desktop/teste.prism'
        self.add_client(req)
        
        
        
        req=GetExpectedTravelTimeRequest()
        req.start_id='WayPoint1'
        req.ltl_task='F "WayPoint2"'
        req.time_of_day='all_day'

        
        d=self.time_client(req)
        rospy.loginfo(d)
        
        req=GeneratePolicyRequest()
        req.start_id='WayPoint2'
        req.ltl_task='F "WayPoint1"'
        req.time_of_day='all_day'
        
        d=self.policy_client(req)
        
        product_mdp=ProductMdp(top_map_mdp,'/home/bruno/tmp/prism/all_day/prod.sta','/home/bruno/tmp/prism/all_day/prod.lab','/home/bruno/tmp/prism/all_day/prod.tra')
        
        product_mdp.write_prism_model('/home/bruno/Desktop/product.prism')
        
        product_mdp.set_policy('/home/bruno/tmp/prism/all_day/adv.tra')



if __name__ == '__main__':
    rospy.init_node('test_client')
    

        
    mdp_planner =  MdpPlanner()
    mdp_planner.main()
