#! /usr/bin/env python

import sys
import rospy
import os


from mdp_plan_exec.prism_client import PrismClient
from mdp_plan_exec.mdp import TopMapMdp, ProductMdp

from strands_executive_msgs.srv import AddMdpModel, GetExpectedTravelTime, UpdateNavStatistics, AddDeleteSpecialWaypoint, AddDeleteSpecialWaypointRequest
#from strands_executive_msgs.srv import GeneratePolicy


from strands_executive_msgs.msg import ExecutePolicyAction, ExecutePolicyFeedback, ExecutePolicyGoal, LearnTravelTimesAction
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal

from actionlib import SimpleActionServer, SimpleActionClient
from std_msgs.msg import String
#from actionlib_msgs import GoalStatus

from strands_navigation_msgs.msg import NavStatistics

    
class MdpPlanner(object):

    def __init__(self,top_map):
    
        self.prism_client=PrismClient()
        self.travel_time_to_node_service = rospy.Service('/mdp_plan_exec/get_expected_travel_time_to_node', GetExpectedTravelTime, self.travel_time_to_node_cb)
        self.add_mdp_service = rospy.Service('/mdp_plan_exec/add_mdp_model', AddMdpModel, self.add_mdp_cb)
        #self.generate_policy=rospy.Service('/mdp_plan_exec/generate_policy', GeneratePolicy, self.policy_cb)
        self.update_nav_statistics=rospy.Service('mdp_plan_exec/update_nav_statistics',UpdateNavStatistics,self.update_cb)
        
        
        rospy.loginfo("Creating topological navigation client.")
        self.top_nav_action_client= SimpleActionClient('topological_navigation', GotoNodeAction)
        self.top_nav_action_client.wait_for_server()
        rospy.loginfo(" ...done")
        rospy.sleep(0.3)
        
        
        self.mdp_navigation_action=SimpleActionServer('mdp_plan_exec/execute_policy', ExecutePolicyAction, execute_cb = self.execute_policy_cb, auto_start = False)
        self.mdp_navigation_action.register_preempt_callback(self.preempt_policy_execution_cb)
        self.mdp_navigation_action.start()
        
        self.learning_travel_times=True
        self.learn_travel_times_action=SimpleActionServer('mdp_plan_exec/learn_travel_times', LearnTravelTimesAction, execute_cb = self.execute_learn_travel_times_cb, auto_start = False)
        self.learn_travel_times_action.register_preempt_callback(self.preempt_learning_cb)
        self.learn_travel_times_action.start()

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
        self.edge_nav_time=0
        self.nav_action_outcome=''
        self.closest_state_subscriber=rospy.Subscriber('/closest_node', String, self.closest_node_cb)
        self.current_state_subscriber=rospy.Subscriber('/current_node', String, self.current_node_cb)
        self.nav_stats_subscriber = rospy.Subscriber('/topological_navigation/Statistics', NavStatistics, self.get_nav_time_cb)
        
        self.forbidden_waypoints=[]
        self.forbidden_waypoints_ltl_string=''
        
        self.safe_waypoints=[]
        self.safe_waypoints_ltl_string=''
        
        self.special_waypoint_handler_service = rospy.Service('/mdp_plan_exec/add_delete_special_node', AddDeleteSpecialWaypoint, self.add_delete_special_waypoint_cb)
        
    

        
    def add_delete_special_waypoint_cb(self,req):
        if req.waypoint_type == AddDeleteSpecialWaypointRequest.FORBIDDEN:
            if req.is_addition:
                self.add_forbidden_waypoint(req.waypoint)
            else:
                self.del_forbidden_waypoint(req.waypoint)
                
        if req.waypoint_type == AddDeleteSpecialWaypointRequest.SAFE:
            if req.is_addition:
                self.add_safe_waypoint(req.waypoint)
            else:
                self.del_safe_waypoint(req.waypoint)
                
        return True
    
    
    def add_forbidden_waypoint(self,waypoint):
        if waypoint in self.forbidden_waypoints:
            rospy.logwarn('Waypoint ' + waypoint + ' already in forbidden waypoint list.')
        else:
            self.forbidden_waypoints.append(waypoint)
            self.set_forbidden_waypoints_ltl_string()
                   
    def del_forbidden_waypoint(self,waypoint):
        if waypoint not in self.forbidden_waypoints:
            rospy.logwarn('Waypoint ' + waypoint + ' not in forbidden waypoint list.')
        else:
            del self.forbidden_waypoints[self.forbidden_waypoints.index(waypoint)]
            self.set_forbidden_waypoints_ltl_string()        
        
        
    def add_safe_waypoint(self,waypoint):
        if waypoint in self.safe_waypoints:
            rospy.logwarn('Waypoint ' + waypoint + ' already in safe waypoint list.')
        else:
            self.safe_waypoints.append(waypoint)
            self.set_safe_waypoints_ltl_string()
    
    def del_safe_waypoint(self,waypoint):
        if waypoint not in self.safe_waypoints:
            rospy.logwarn('Waypoint ' + waypoint + ' not in safe waypoint list.')
        else:
            del self.safe_waypoints[self.safe_waypoints.index(waypoint)]
            self.set_safe_waypoints_ltl_string()    
        
        
    
    def set_forbidden_waypoints_ltl_string(self):
        self.forbidden_waypoints_ltl_string=''
        
        for i in range(0,len(self.forbidden_waypoints)):
            self.forbidden_waypoints_ltl_string=self.forbidden_waypoints_ltl_string + '"' + self.forbidden_waypoints[i] + '" & !'
        
        if not self.forbidden_waypoints_ltl_string=='':
            self.forbidden_waypoints_ltl_string='(!' + self.forbidden_waypoints_ltl_string[:-4] + ')'
        
        print self.forbidden_waypoints_ltl_string
            

    def set_safe_waypoints_ltl_string(self):
        self.safe_waypoints_ltl_string=''
        
        for i in range(0,len(self.safe_waypoints)):
            self.safe_waypoints_ltl_string=self.safe_waypoints_ltl_string  + '"' + self.safe_waypoints[i] + '"'  + ' | '
        
        if not self.safe_waypoints_ltl_string=='':
            self.safe_waypoints_ltl_string='(' + self.safe_waypoints_ltl_string[:-3] + ')'
            
        print self.safe_waypoints_ltl_string
    
        
    def travel_time_to_node_cb(self,req):
        starting_node= req.start_id
        self.top_map_mdp.set_initial_state_from_name(starting_node)
        self.update_current_top_mdp(req.time_of_day,self.mdp_prism_file)
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
        self.update_current_top_mdp(req.time_of_day,self.mdp_prism_file)
        return True
    
    
    def update_current_top_mdp(self,time_of_day,mdp_file):
        self.top_map_mdp.update_nav_statistics()
        self.top_map_mdp.write_prism_model(mdp_file)
        result=self.prism_client.update_model(time_of_day,self.mdp_prism_file)
    
    
    def execute_learn_travel_times_cb(self,goal):
        rospy.set_param('/topological_navigation/mode', 'Node_by_node')
        self.learning_travel_times=True
        rospy.Timer(rospy.Duration(goal.timeout), self.finish_learning_callback)
        n_successive_fails=0
        
        while self.learning_travel_times:
            if self.current_node == 'none':
                self.top_map_mdp.set_initial_state_from_name(self.closest_node) 
            else:
                self.top_map_mdp.set_initial_state_from_name(self.current_node)
            
            current_waypoint=self.top_map_mdp.initial_state
            current_waypoint_trans=self.top_map_mdp.transitions[current_waypoint]
            current_trans_count=self.top_map_mdp.transitions_transversal_count[current_waypoint]
            current_min=-1
            current_min_index=-1
            for i in range(0,self.top_map_mdp.n_actions):
                if current_waypoint_trans[i] is not False:
                    if current_min==-1:
                        current_min=current_trans_count[i]
                        current_min_index=i
                    elif current_trans_count[i]<current_min:
                        current_min=current_trans_count[i]
                        current_min_index=i
            current_action=self.top_map_mdp.actions[current_min_index]
            top_nav_goal=GotoNodeGoal()
            top_nav_goal.target=current_action.split('_')[2]
            self.top_nav_action_client.send_goal(top_nav_goal)
            self.top_nav_action_client.wait_for_result()
            
            if self.nav_action_outcome=='fatal':
                n_successive_fails=n_successive_fails+1
            else:
                n_successive_fails=0
            
            if n_successive_fails>1:
                self.update_current_top_mdp('all_day',self.mdp_prism_file)
                self.learn_travel_times_action.set_aborted()
                return
            
            self.top_map_mdp.transitions_transversal_count[current_waypoint][current_min_index]+=1

        
        
    def finish_learning_callback(self,event):
        self.update_current_top_mdp('all_day',self.mdp_prism_file)
        self.learn_travel_times_action.set_succeeded()
        self.learning_travel_times=False
        
    def preempt_learning_cb(self):
        self.update_current_top_mdp('all_day',self.mdp_prism_file)
        self.top_nav_action_client.cancel_all_goals()
        self.learn_travel_times_action.set_preempted()


    
   
    def execute_policy_cb(self,goal):
        
        rospy.set_param('/topological_navigation/mode', 'Node_to_IZ')
        if self.current_node == 'none':
            self.top_map_mdp.set_initial_state_from_name(self.closest_node) 
        else:
            self.top_map_mdp.set_initial_state_from_name(self.current_node)
        
        self.update_current_top_mdp(goal.time_of_day,self.mdp_prism_file)
        if goal.task_type==ExecutePolicyGoal.GOTO_WAYPOINT:
            if self.forbidden_waypoints==[]:
                specification='R{"time"}min=? [ (F "' + goal.target_id + '") ]'
            else:
                specification='R{"time"}min=? [ (' + self.forbidden_waypoints_ltl_string + ' U "' + goal.target_id + '") ]'
        elif goal.task_type==ExecutePolicyGoal.LEAVE_FORBIDDEN_AREA:
            if self.forbidden_waypoints==[]:
                rospy.logerr("No forbidden waypoints defined. Nothing to leave.")
                self.mdp_navigation_action.set_aborted()
                return
            elif self.closest_node not in self.forbidden_waypoints:
                rospy.logerr(self.closest_node + " is not a forbidden waypoint. Staying here.")
                self.mdp_navigation_action.set_aborted()
                return
            else:
                specification='R{"time"}min=? [ (F ' + self.forbidden_waypoints_ltl_string + ') ]'
        elif goal.task_type==ExecutePolicyGoal.GOTO_CLOSEST_SAFE_WAYPOINT:
            if self.safe_waypoints==[]:
                rospy.logerr("No safe waypoints defined. Nowhere to go to.")
                self.mdp_navigation_action.set_aborted()
                return
            elif self.current_node  in self.safe_waypoints:
                rospy.logerr(self.closest_node + " is already a safe waypoint. Staying here.")
                self.mdp_navigation_action.set_aborted()
                return
            else:
                specification='R{"time"}min=? [ (F ' + self.safe_waypoints_ltl_string + ') ]'
                
            
        feedback=ExecutePolicyFeedback()
        feedback.expected_time=float(self.prism_client.get_policy(goal.time_of_day,specification))
        self.mdp_navigation_action.publish_feedback(feedback)
        result_dir=self.directory + '/' + goal.time_of_day 
        product_mdp=ProductMdp(self.top_map_mdp,result_dir + '/prod.sta',result_dir + '/prod.lab',result_dir + '/prod.tra')
        product_mdp.set_policy(result_dir + '/adv.tra')
        
        current_mdp_state=product_mdp.initial_state
        n_successive_fails=0
        while current_mdp_state not in product_mdp.goal_states:
            current_action=product_mdp.policy[current_mdp_state]
            expected_edge_transversal_time=product_mdp.get_expected_edge_transversal_time(current_mdp_state,current_action)
            print product_mdp.get_total_transversals(current_mdp_state,current_action)
            print expected_edge_transversal_time
            top_nav_goal=GotoNodeGoal()
            top_nav_goal.target=current_action.split('_')[2]
            self.top_nav_action_client.send_goal(top_nav_goal)
            self.top_nav_action_client.wait_for_result()
            if self.current_node == 'none':
                current_mdp_state=product_mdp.get_new_state(current_mdp_state,current_action,self.closest_node)
            else:
                current_mdp_state=product_mdp.get_new_state(current_mdp_state,current_action,self.current_node)
             
            
            
            if current_mdp_state==-1:
                rospy.logerr('State transition is not in model!')
                self.mdp_navigation_action.set_aborted()
                return
            elif self.edge_nav_time>2*expected_edge_transversal_time and product_mdp.get_total_transversals(current_mdp_state,current_action)>1:
                rospy.logwarn("took too long transvesing the edge")
                
            if self.nav_action_outcome=='fatal':
                n_successive_fails=n_successive_fails+1
            else:
                n_successive_fails=0
            
            if n_successive_fails>1:
                self.mdp_navigation_action.set_aborted()
                return
                
        
        
        #go to the exact goal waypoint pose
        
        
        self.mdp_navigation_action.set_succeeded()
            
       
    def preempt_policy_execution_cb(self):
        self.top_nav_action_client.cancel_all_goals()
        self.mdp_navigation_action.set_preempted()
    
    def closest_node_cb(self,msg):
        self.closest_node=msg.data
        
    def current_node_cb(self,msg):
        self.current_node=msg.data

    def get_nav_time_cb(self,msg):   
        self.edge_nav_time=msg.operation_time-msg.time_to_waypoint
        self.nav_action_outcome=msg.status
    
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
    
    
