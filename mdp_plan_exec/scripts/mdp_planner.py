#! /usr/bin/env python

import sys
import rospy
import os


from mdp_plan_exec.prism_client import PrismClient
from mdp_plan_exec.mdp import TopMapMdp, ProductMdp

from strands_executive_msgs.srv import AddMdpModel
from strands_executive_msgs.srv import GetExpectedTravelTime
#from strands_executive_msgs.srv import GeneratePolicy
from strands_executive_msgs.srv import UpdateNavStatistics

from strands_executive_msgs.msg import ExecutePolicyAction, ExecutePolicyFeedback
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal

from actionlib import SimpleActionServer, SimpleActionClient
from std_msgs.msg import String
#from actionlib_msgs import GoalStatus

    
class MdpPlanner(object):

    def __init__(self,top_map):
    
        self.prism_client=PrismClient()
        self.travel_time_to_node_service = rospy.Service('/mdp_plan_exec/get_expected_travel_time_to_node', GetExpectedTravelTime, self.travel_time_to_node_cb)
        self.add_mdp_service = rospy.Service('/mdp_plan_exec/add_mdp_model', AddMdpModel, self.add_mdp_cb)
        #self.generate_policy=rospy.Service('/mdp_plan_exec/generate_policy', GeneratePolicy, self.policy_cb)
        self.update_nav_statistics=rospy.Service('mdp_plan_exec/update_nav_statistics',UpdateNavStatistics,self.update_cb)
        
        self.mdp_navigation_action=SimpleActionServer('mdp_plan_exec/execute_policy', ExecutePolicyAction, execute_cb = self.execute_policy_cb, auto_start = False)
        self.mdp_navigation_action.start()

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
        
        self.closest_node=None
        self.current_node=None
        self.closest_state_subscriber=rospy.Subscriber('/closest_node', String, self.closest_node_cb)
        self.current_state_subscriber=rospy.Subscriber('/current_node', String, self.current_node_cb)
        
        self.forbidden_nodes=[]
        self.forbidden_nodes_ltl_string=''

        
    
        
    def travel_time_to_node_cb(self,req):
        starting_node= req.start_id
        self.top_map_mdp.set_initial_state_from_name(starting_node)
        self.top_map_mdp.write_prism_model(self.mdp_prism_file)
        result=self.prism_client.update_model(req.time_of_day,self.mdp_prism_file)
        specification='R{"time"}min=? [ ( F "' + req.target_id + '") ]'
        result=self.prism_client.check_model(req.time_of_day,specification)
        result=float(result)
        return result
        
    def add_mdp_cb(self,req):
        self.prism_client.add_model(req.time_of_day,req.mdp_file)
        return True
     
    
    #def policy_cb(self,req):
        #starting_node= req.start_id
        #self.top_map_mdp.set_initial_state_from_name(starting_node)
        #self.top_map_mdp.write_prism_model(self.mdp_prism_file)
        #result=self.prism_client.update_model(req.time_of_day,self.mdp_prism_file)
        #specification='R{"time"}min=? [ (' + req.ltl_task + ') ]'
        #result=self.prism_client.get_policy(req.time_of_day,specification)
        #return result
        
    def update_cb(self,req): 
        self.top_map_mdp.update_nav_statistics()
        self.top_map_mdp.write_prism_model(self.mdp_prism_file)
        result=self.prism_client.update_model(req.time_of_day,self.mdp_prism_file)
        return True
        
   
    def execute_policy_cb(self,goal):
        rospy.loginfo("Creating monitored navigation client.")
        top_nav_action_client= SimpleActionClient('topological_navigation', GotoNodeAction)
        top_nav_action_client.wait_for_server()
        rospy.loginfo(" ...done")
        rospy.sleep(0.3)
   
        if self.current_node == 'none':
            self.top_map_mdp.set_initial_state_from_name(self.closest_node) 
        else:
            self.top_map_mdp.set_initial_state_from_name(self.current_node)
            
        self.top_map_mdp.write_prism_model(self.mdp_prism_file)
        result=self.prism_client.update_model(goal.time_of_day,self.mdp_prism_file)
        if self.forbidden_nodes==[]:
            specification='R{"time"}min=? [ (F "' + goal.target_id + '") ]'
        else:
            specification='R{"time"}min=? [ (' + self.forbidden_nodes_ltl_string + ' U "' + goal.target_id + '") ]'
        feedback=ExecutePolicyFeedback()
        feedback.expected_time=float(self.prism_client.get_policy(goal.time_of_day,specification))
        self.mdp_navigation_action.publish_feedback(feedback)
        result_dir=self.directory + '/' + goal.time_of_day 
        product_mdp=ProductMdp(self.top_map_mdp,result_dir + '/prod.sta',result_dir + '/prod.lab',result_dir + '/prod.tra')
        product_mdp.set_policy(result_dir + '/adv.tra')
        
        current_mdp_state=product_mdp.initial_state
        while current_mdp_state not in product_mdp.goal_states:
            current_action=product_mdp.policy[current_mdp_state]
            top_nav_goal=GotoNodeGoal()
            top_nav_goal.target=current_action.split('_')[2]
            top_nav_action_client.send_goal(top_nav_goal)
            top_nav_action_client.wait_for_result()
            if self.current_node == 'none':
                current_mdp_state=product_mdp.get_new_state(current_mdp_state,current_action,self.closest_node)
            else:
                current_mdp_state=product_mdp.get_new_state(current_mdp_state,current_action,self.current_node)
                
            
            if current_mdp_state==-1:
                rospy.logerror('State transition is not in model!')
                self.mdp_navigation_action.set_aborted()
        
        self.mdp_navigation_action.set_succeeded()
            
       
    
    
    def closest_node_cb(self,msg):
        self.closest_node=msg.data
        
    def current_node_cb(self,msg):
        self.current_node=msg.data

    
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
    
    
