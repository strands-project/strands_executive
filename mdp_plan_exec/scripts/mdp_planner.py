#! /usr/bin/env python

import sys
import rospy
import os


#from mdp_plan_exec.prism_client import PrismClient
#from mdp_plan_exec.mdp import TopMapMdp, ProductMdp
from mdp_plan_exec.mdp import ProductMdp
from mdp_plan_exec.prism_mdp_manager import PrismMdpManager

from strands_executive_msgs.srv import AddMdpModel, GetExpectedTravelTime, UpdateNavStatistics, AddDeleteSpecialWaypoint, AddDeleteSpecialWaypointRequest


from strands_executive_msgs.msg import ExecutePolicyAction, ExecutePolicyFeedback, ExecutePolicyGoal, LearnTravelTimesAction
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal

from actionlib import SimpleActionServer, SimpleActionClient
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatus

from strands_navigation_msgs.msg import NavStatistics, ExecutePolicyModeAction, ExecutePolicyModeFeedback, ExecutePolicyModeGoal

from mongodb_store.message_store import MessageStoreProxy
from robblog.msg import RobblogEntry
import robblog.utils
from sensor_msgs.msg import Image
import datetime

    
class MdpPlanner(object):

    def __init__(self,top_map):
    
        self.exp_times_handler=PrismMdpManager(8085,'exp_times', top_map)
        self.policy_handler=PrismMdpManager(8086,'policy',top_map)
        
        
        self.travel_time_to_node_service = rospy.Service('/mdp_plan_exec/get_expected_travel_time_to_node', GetExpectedTravelTime, self.travel_time_to_node_cb)
        self.add_mdp_service = rospy.Service('/mdp_plan_exec/add_mdp_model', AddMdpModel, self.add_mdp_cb)
        #self.generate_policy=rospy.Service('/mdp_plan_exec/generate_policy', GeneratePolicy, self.policy_cb)
        self.update_nav_statistics=rospy.Service('mdp_plan_exec/update_nav_statistics',UpdateNavStatistics,self.update_cb)
        
        
        rospy.loginfo("Creating topological navigation client.")
        self.top_nav_action_client= SimpleActionClient('topological_navigation', GotoNodeAction)
        self.top_nav_action_client.wait_for_server()
        rospy.loginfo(" ...done")
        rospy.sleep(0.3)
        
        self.current_prod_mdp_state=None
        self.policy_exec_preempted = False
        rospy.loginfo("Creating topological navigation client.")
        self.top_nav_policy_exec= SimpleActionClient('/topological_navigation/execute_policy_mode', ExecutePolicyModeAction)
        self.top_nav_policy_exec.wait_for_server()
        rospy.loginfo(" ...done")
        rospy.sleep(0.3)        
        
        #rospy.loginfo("Creating monitored navigation client.")
        #self.mon_nav_action_client= SimpleActionClient('monitored_navigation', MonitoredNavigationAction)
        #self.mon_nav_action_client.wait_for_server()
        #rospy.loginfo(" ...done")
        #rospy.sleep(0.3)
        
        
        self.executing_policy=False
        self.mdp_navigation_action=SimpleActionServer('mdp_plan_exec/execute_policy', ExecutePolicyAction, execute_cb = self.execute_policy_cb, auto_start = False)
        self.mdp_navigation_action.register_preempt_callback(self.preempt_policy_execution_cb)
        self.mdp_navigation_action.start()
        
        self.learning_travel_times=False
        self.learn_travel_times_action=SimpleActionServer('mdp_plan_exec/learn_travel_times', LearnTravelTimesAction, execute_cb = self.execute_learn_travel_times_cb, auto_start = False)
        self.learn_travel_times_action.register_preempt_callback(self.preempt_learning_cb)
        self.learn_travel_times_action.start()

        #self.top_map_mdp=TopMapMdp(top_map)
        #self.top_map_mdp.update_nav_statistics()
        
        #self.directory = os.path.expanduser("~") + '/tmp/prism'
        #try:
            #os.makedirs(self.directory)
        #except OSError as ex:
            #print 'error creating PRISM directory:',  ex
            
        #self.mdp_prism_file=self.directory+'/'+top_map+'.prism'    
        
        #self.top_map_mdp.write_prism_model(self.mdp_prism_file)
        
        #self.prism_client.add_model('all_day',self.mdp_prism_file)
        
        self.closest_node=None
        self.current_node=None
        self.nav_action_outcome=''
        self.closest_state_subscriber=rospy.Subscriber('/closest_node', String, self.closest_node_cb)
        self.current_state_subscriber=rospy.Subscriber('/current_node', String, self.current_node_cb)
        self.nav_stats_subscriber = rospy.Subscriber('/topological_navigation/Statistics', NavStatistics, self.get_nav_status_cb)
        
        #self.nonitored_nav_result=None
        #self.monitored_nav_sub=rospy.Subscriber('/monitored_navigation/result', MonitoredNavigationActionResult, self.get_monitored_nav_status_cb)
        
        self.get_to_exact_pose_timeout=120 #60 secs
        
        
        self.forbidden_waypoints=[]
        self.forbidden_waypoints_ltl_string=''
        
        self.safe_waypoints=[]
        self.safe_waypoints_ltl_string=''
        
        self.special_waypoint_handler_service = rospy.Service('/mdp_plan_exec/add_delete_special_node', AddDeleteSpecialWaypoint, self.add_delete_special_waypoint_cb)
        
        self.msg_store_blog = MessageStoreProxy(collection='robblog')
        self.origin_waypoint=''
        self.target_waypoint=''
        #self.last_stuck_image=None

        
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
 
 
 
#-------------------------updating models for both expected time and policy generation 
    def add_mdp_cb(self,req):
        self.exp_times_handler.prism_client.add_model(req.time_of_day,req.mdp_file)
        self.policy_handler.prism_client.add_model(req.time_of_day,req.mdp_file)
        return True
     
    
    def update_cb(self,req): 
        self.exp_times_handler.update_current_top_mdp(req.time_of_day)
        self.policy_handler.update_current_top_mdp(req.time_of_day)
        return True
 
 
#-------------------------expected times     
    def travel_time_to_node_cb(self,req):
        starting_node= req.start_id
        self.exp_times_handler.top_map_mdp.set_initial_state_from_name(starting_node)
        self.exp_times_handler.update_current_top_mdp(req.time_of_day, False)
        specification='R{"time"}min=? [ ( F "' + req.target_id + '") ]'
        result=self.exp_times_handler.prism_client.check_model(req.time_of_day,specification)
        result=float(result)
        return result
        
        

#-------------------------policy generation/execution    
    def execute_learn_travel_times_cb(self,goal):
        
        if self.executing_policy:
            self.preempt_policy_execution_cb()
        
        rospy.set_param('/topological_navigation/mode', 'Node_by_node')
        self.learning_travel_times=True
        timer=rospy.Timer(rospy.Duration(goal.timeout), self.finish_learning_callback,oneshot=True)
        n_successive_fails=0
        
        while self.learning_travel_times:
            if self.current_node == 'none' or self.current_node is None:
                self.policy_handler.top_map_mdp.set_initial_state_from_name(self.closest_node) 
            else:
                self.policy_handler.top_map_mdp.set_initial_state_from_name(self.current_node)
            current_waypoint=self.policy_handler.top_map_mdp.initial_state
            current_waypoint_trans=self.policy_handler.top_map_mdp.transitions[current_waypoint]
            current_trans_count=self.policy_handler.top_map_mdp.transitions_transversal_count[current_waypoint]
            current_min=-1
            current_min_index=-1
            for i in range(0,self.policy_handler.top_map_mdp.n_actions):
                if current_waypoint_trans[i] is not False:
                    if current_min==-1:
                        current_min=current_trans_count[i]
                        current_min_index=i
                    elif current_trans_count[i]<current_min:
                        current_min=current_trans_count[i]
                        current_min_index=i
            current_action=self.policy_handler.top_map_mdp.actions[current_min_index]
            top_nav_goal=GotoNodeGoal()
            top_nav_goal.target=current_action.split('_')[2]
            self.top_nav_action_client.send_goal(top_nav_goal)
            self.top_nav_action_client.wait_for_result()
            
            if self.nav_action_outcome=='fatal' or self.nav_action_outcome=='failed':
                n_successive_fails=n_successive_fails+1
            else:
                n_successive_fails=0
            
            if n_successive_fails>10:
                self.policy_handler.update_current_top_mdp('all_day')
                self.learn_travel_times_action.set_aborted()
                return
            
            self.policy_handler.top_map_mdp.transitions_transversal_count[current_waypoint][current_min_index]+=1
            
        self.exp_times_handler.update_current_top_mdp("all_day")    
        timer.shutdown()    

        
        
    def finish_learning_callback(self,event):
        self.policy_handler.update_current_top_mdp('all_day')
        self.learn_travel_times_action.set_succeeded()
        self.learning_travel_times=False
        
    def preempt_learning_cb(self):
        self.learning_travel_times=False
        self.policy_handler.update_current_top_mdp('all_day')
        self.top_nav_action_client.cancel_all_goals()
        self.learn_travel_times_action.set_preempted()
    
    
    def generate_prod_mdp_policy(self,specification, time_of_day):
        #update initial state
        if self.current_node == 'none' or self.current_node is None:
            self.policy_handler.top_map_mdp.set_initial_state_from_name(self.closest_node) 
        else:
            self.policy_handler.top_map_mdp.set_initial_state_from_name(self.current_node)
        self.policy_handler.update_current_top_mdp(time_of_day)
        
        feedback=ExecutePolicyFeedback()
        feedback.expected_time=float(self.policy_handler.prism_client.get_policy(time_of_day,specification))
        self.mdp_navigation_action.publish_feedback(feedback)
        if feedback.expected_time==float("inf"):            
            self.policy_handler.product_mdp=None
        else:
            result_dir=self.policy_handler.get_working_dir() + '/' + time_of_day 
            self.policy_handler.product_mdp=ProductMdp(self.policy_handler.top_map_mdp,result_dir + '/prod.sta',result_dir + '/prod.lab',result_dir + '/prod.tra')
            self.policy_handler.product_mdp.set_policy(result_dir + '/adv.tra')
    

    def execute_policy_cb(self,goal):
        
        if self.learning_travel_times:
            self.preempt_learning_cb()
                
        
        if goal.target_id == self.closest_node:
            rospy.loginfo("Already in goal influence area. Navigating to exact pose...")
            self.top_nav_policy_exec.send_goal(ExecutePolicyModeGoal(), feedback_cb = self.top_nav_feedback_cb)
            status=self.top_nav_policy_exec.get_state()
            while (status==GoalStatus.PENDING or status==GoalStatus.ACTIVE):
                self.top_nav_policy_exec.wait_for_result(rospy.Duration(0.2))
                status= self.top_nav_policy_exec.get_state()
                if  self.policy_exec_preempted:
                    self.execute_policy_service_preempt()
                    return
            if status != GoalStatus.SUCCEEDED:
                rospy.logerr("Policy mode execution finished with status " + str(status) + ". Aborting...")
                self.executing_policy=False
                self.mdp_navigation_action.set_aborted()
                return
            else:
                self.mdp_navigation_action.set_succeeded()
                return
            
        
        if goal.task_type==ExecutePolicyGoal.GOTO_WAYPOINT:
            if goal.target_id in self.forbidden_waypoints:
                rospy.logerr("The goal is a forbidden waypoint. Aborting")
                self.mdp_navigation_action.set_aborted()
                return
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
                
            

        
        
        self.generate_prod_mdp_policy(specification,goal.time_of_day)
        if self.policy_handler.product_mdp is None:
            rospy.logerr("The goal is unattainable. Aborting...")
            self.mdp_navigation_action.set_aborted()
            return
  
        self.current_prod_mdp_state=self.policy_handler.product_mdp.initial_state 
        current_policy_mode = self.policy_handler.product_mdp.get_current_policy_mode(self.current_prod_mdp_state)

   
        if  self.policy_exec_preempted:
            self.execute_policy_service_preempt()
            return
   
        self.executing_policy=True
        replanned=False
        while self.current_prod_mdp_state not in self.policy_handler.product_mdp.goal_states and self.executing_policy and not rospy.is_shutdown():
            self.top_nav_policy_exec.send_goal(ExecutePolicyModeGoal(route = current_policy_mode), feedback_cb = self.top_nav_feedback_cb)
            status=self.top_nav_policy_exec.get_state()
            self.finishing_policy_mode_execution=False
            while (status==GoalStatus.PENDING or status==GoalStatus.ACTIVE) and self.executing_policy:
                #check that mdp still knows where it is
                if self.current_prod_mdp_state==None:
                    rospy.logwarn('State transition is not in MDP model! Replanning...')
                    replanned = True
                    self.top_nav_policy_exec.cancel_all_goals()
                    self.generate_prod_mdp_policy(specification,goal.time_of_day)
                    if self.policy_handler.product_mdp is None:
                        rospy.logerr("The goal is unattainable. Aborting...")
                        self.executing_policy=False
                        self.mdp_navigation_action.set_aborted()
                        return
                    self.current_prod_mdp_state=self.policy_handler.product_mdp.initial_state
                self.top_nav_policy_exec.wait_for_result(rospy.Duration(0.2))
                status=self.top_nav_policy_exec.get_state()
            
            if not replanned:               
                if  self.policy_exec_preempted:
                    self.execute_policy_service_preempt()
                    return
                    
                if status != GoalStatus.SUCCEEDED:
                    rospy.logerr("Policy mode execution finished with status " + str(status) + ". Aborting...")
                    self.executing_policy=False
                    self.mdp_navigation_action.set_aborted()
                    return
    
                if self.executing_policy and self.current_prod_mdp_state not in self.policy_handler.product_mdp.goal_states:
                    current_policy_mode = self.policy_handler.product_mdp.get_current_policy_mode(self.current_prod_mdp_state)
                        
                if self.policy_exec_preempted:
                    self.execute_policy_service_preempt()
                    return
            else:
                replanned=False
                   
        self.executing_policy = False                            
        
        self.exp_times_handler.update_current_top_mdp(goal.time_of_day)

        if self.policy_exec_preempted:
            self.execute_policy_service_preempt()
            return
        self.mdp_navigation_action.set_succeeded()
        
   
    def top_nav_feedback_cb(self,feedback):
        if not self.finishing_policy_mode_execution:
            self.current_node = feedback.route_status
            current_action=self.policy_handler.product_mdp.policy[self.current_prod_mdp_state]
            self.current_prod_mdp_state = self.policy_handler.product_mdp.get_new_state(self.current_prod_mdp_state,current_action, self.current_node)
            print self.current_prod_mdp_state
            if self.current_prod_mdp_state in self.policy_handler.product_mdp.goal_states:
                self.finishing_policy_mode_execution=True
        
        
    def preempt_policy_execution_cb(self):
        self.policy_exec_preempted = True
        self.executing_policy=False
        
        
        
    def execute_policy_service_preempt(self):
        self.top_nav_policy_exec.cancel_all_goals()
        self.policy_exec_preempted = False
        self.mdp_navigation_action.set_preempted()
   
    #def execute_policy_cb(self,goal):
        
        #if self.learning_travel_times:
            #self.preempt_learning_cb()
        
        
        #rospy.set_param('/topological_navigation/mode', 'Node_to_IZ')
        #if self.current_node == 'none' or self.current_node is None:
            #self.policy_handler.top_map_mdp.set_initial_state_from_name(self.closest_node) 
        #else:
            #self.policy_handler.top_map_mdp.set_initial_state_from_name(self.current_node)
        #self.policy_handler.update_current_top_mdp(goal.time_of_day)
        #if goal.task_type==ExecutePolicyGoal.GOTO_WAYPOINT:
            #if goal.target_id in self.forbidden_waypoints:
                #rospy.logerr("The goal is a forbidden waypoint. Aborting")
                #self.mdp_navigation_action.set_aborted()
                #return
            #if self.forbidden_waypoints==[]:
                #specification='R{"time"}min=? [ (F "' + goal.target_id + '") ]'
            #else:
                #specification='R{"time"}min=? [ (' + self.forbidden_waypoints_ltl_string + ' U "' + goal.target_id + '") ]'
        #elif goal.task_type==ExecutePolicyGoal.LEAVE_FORBIDDEN_AREA:
            #if self.forbidden_waypoints==[]:
                #rospy.logerr("No forbidden waypoints defined. Nothing to leave.")
                #self.mdp_navigation_action.set_aborted()
                #return
            #elif self.closest_node not in self.forbidden_waypoints:
                #rospy.logerr(self.closest_node + " is not a forbidden waypoint. Staying here.")
                #self.mdp_navigation_action.set_aborted()
                #return
            #else:
                #specification='R{"time"}min=? [ (F ' + self.forbidden_waypoints_ltl_string + ') ]'
        #elif goal.task_type==ExecutePolicyGoal.GOTO_CLOSEST_SAFE_WAYPOINT:
            #if self.safe_waypoints==[]:
                #rospy.logerr("No safe waypoints defined. Nowhere to go to.")
                #self.mdp_navigation_action.set_aborted()
                #return
            #elif self.current_node  in self.safe_waypoints:
                #rospy.logerr(self.closest_node + " is already a safe waypoint. Staying here.")
                #self.mdp_navigation_action.set_aborted()
                #return
            #else:
                #specification='R{"time"}min=? [ (F ' + self.safe_waypoints_ltl_string + ') ]'
                
            
        #feedback=ExecutePolicyFeedback()
        #feedback.expected_time=float(self.policy_handler.prism_client.get_policy(goal.time_of_day,specification))
        #self.mdp_navigation_action.publish_feedback(feedback)
        #if feedback.expected_time==float("inf"):
            #rospy.logerr("The goal is unattainable with the current forbidden nodes. Aborting...")
            #self.mdp_navigation_action.set_aborted()
            #return
        #result_dir=self.policy_handler.get_working_dir() + '/' + goal.time_of_day 
        #product_mdp=ProductMdp(self.policy_handler.top_map_mdp,result_dir + '/prod.sta',result_dir + '/prod.lab',result_dir + '/prod.tra')
        #product_mdp.set_policy(result_dir + '/adv.tra')
        
        #self.executing_policy=True
        #current_mdp_state=product_mdp.initial_state
        #if current_mdp_state in product_mdp.goal_states:
            #rospy.set_param('/topological_navigation/mode', 'Normal')
            #top_nav_goal=GotoNodeGoal()
            #top_nav_goal.target=goal.target_id
            #self.top_nav_action_client.send_goal(top_nav_goal)
            #self.top_nav_action_client.wait_for_result()
            #result=self.top_nav_action_client.get_state()
            #if result==GoalStatus.SUCCEEDED:
                #self.mdp_navigation_action.set_succeeded()
            #if result==GoalStatus.ABORTED:
                #rospy.logerr("Failure in getting to exact pose in goal waypoint")
                #self.mdp_navigation_action.set_aborted()
            #if result==GoalStatus.PREEMPTED:
                #self.mdp_navigation_action.set_preempted()
            #return

            
            
        
        #n_successive_fails=0
        #while current_mdp_state not in product_mdp.goal_states and self.executing_policy and not rospy.is_shutdown():
            #current_action=product_mdp.policy[current_mdp_state]
            #expected_edge_transversal_time=product_mdp.get_expected_edge_transversal_time(current_mdp_state,current_action)
            #top_nav_goal=GotoNodeGoal()
            #split_action=current_action.split('_')
            #self.origin_waypoint=split_action[1]
            #self.target_waypoint=split_action[2]
            #top_nav_goal.target=self.target_waypoint
            #timer=rospy.Timer(rospy.Duration(4*expected_edge_transversal_time), self.unexpected_trans_time_cb,oneshot=True)
            #self.top_nav_action_client.send_goal(top_nav_goal)
            #self.top_nav_action_client.wait_for_result()
            #if self.current_node == 'none' or self.current_node is None:
                #current_mdp_state=product_mdp.get_new_state(current_mdp_state,current_action,self.closest_node)
            #else:
                #current_mdp_state=product_mdp.get_new_state(current_mdp_state,current_action,self.current_node)
             
            #timer.shutdown()
            
            
            #if current_mdp_state==-1 and self.executing_policy:
                #rospy.logwarn('State transition is not in MDP model! Replanning...')
                #self.mon_nav_action_client.cancel_all_goals()
                #self.top_nav_action_client.cancel_all_goals()
                #if self.current_node == 'none' or self.current_node is None:
                    #self.policy_handler.top_map_mdp.set_initial_state_from_name(self.closest_node) 
                #else:
                    #self.policy_handler.top_map_mdp.set_initial_state_from_name(self.current_node)
                #self.policy_handler.update_current_top_mdp(goal.time_of_day)
                #feedback.expected_time=float(self.policy_handler.prism_client.get_policy(goal.time_of_day,specification))
                #self.mdp_navigation_action.publish_feedback(feedback)
                #if feedback.expected_time==float("inf"):
                    #rospy.logerr("The goal is unattainable with the current forbidden nodes. Aborting...")
                    #self.mdp_navigation_action.set_aborted()
                    #return
                #product_mdp=ProductMdp(self.policy_handler.top_map_mdp,result_dir + '/prod.sta',result_dir + '/prod.lab',result_dir + '/prod.tra')
                #product_mdp.set_policy(result_dir + '/adv.tra')
                #current_mdp_state=product_mdp.initial_state
                #if current_mdp_state in product_mdp.goal_states:
                    #rospy.set_param('/topological_navigation/mode', 'Normal')
                    #top_nav_goal=GotoNodeGoal()
                    #top_nav_goal.target=goal.target_id
                    #self.top_nav_action_client.send_goal(top_nav_goal)

            #if self.nav_action_outcome=='fatal' or self.nav_action_outcome=='failed':
                #n_successive_fails=n_successive_fails+1
            #else:
                #n_successive_fails=0
            
            #if n_successive_fails>4:
                #rospy.logerr("Five successive fails in topological navigation. Aborting...")
                #self.executing_policy=False
                #self.mon_nav_action_client.cancel_all_goals()
                #self.top_nav_action_client.cancel_all_goals()
                #self.mdp_navigation_action.set_aborted()
                #return
                
        
        #self.exp_times_handler.update_current_top_mdp(goal.time_of_day)
        
        #self.monitored_nav_result=None
        #timeout_counter=0
        #while self.monitored_nav_result is None and self.executing_policy and timeout_counter < self.get_to_exact_pose_timeout:     
            #rospy.sleep(0.5)
            #timeout_counter=timeout_counter+1
        
        
        #if self.executing_policy:
            #self.executing_policy=False
            #if self.monitored_nav_result==GoalStatus.PREEMPTED:
                #self.mdp_navigation_action.set_preempted()
                #return
            #if self.monitored_nav_result==GoalStatus.SUCCEEDED and self.current_node == goal.target_id:
                #self.mdp_navigation_action.set_succeeded()
                #return
            #if self.monitored_nav_result==GoalStatus.ABORTED or self.monitored_nav_result is None or not self.current_node == goal.target_id:
                #rospy.logerr("Failure in getting to exact pose in goal waypoint")
                #self.mdp_navigation_action.set_aborted()
                #return
            

 
    #def get_monitored_nav_status_cb(self,msg):
        #self.monitored_nav_result=msg.status.status
        
 
 
            
    #def preempt_policy_execution_cb(self):
        #self.executing_policy=False
        #self.mon_nav_action_client.cancel_all_goals()
        #self.top_nav_action_client.cancel_all_goals()
        #self.mdp_navigation_action.set_preempted()
        
        
    def unexpected_trans_time_cb(self,event):
        last_stuck_image = None
        image_topic =  '/head_xtion/rgb/image_color'
        #image_topic =  '/head_xtion/rgb/image_mono'  #simulation topic
        
        #count = 0
        #while self.last_stuck_image == None and  not rospy.is_shutdown() and count < 10:
            #rospy.loginfo('waiting for image of possible blocked path %s' % count)
            #count += 1
            #rospy.sleep(1)
            
        last_stuck_image=rospy.wait_for_message(image_topic, Image , timeout=10.0)
            
        e = RobblogEntry(title=datetime.datetime.now().strftime("%H:%M:%S") + ' - Possible Blocked Path')
        e.body = 'It took me a lot more time to go between ' + self.origin_waypoint + ' and ' + self.target_waypoint + ' than I was expecting. Something might be blocking the way.'
            
        if last_stuck_image != None:
            img_id = self.msg_store_blog.insert(last_stuck_image)
            rospy.loginfo('adding possible blockage image to blog post')
            e.body += ' Here is what I saw: \n\n![Image of the door](ObjectID(%s))' % img_id
        self.msg_store_blog.insert(e)
        
        
    #def img_callback(self, img):
        #self.last_stuck_image = img
    
        
    
    def closest_node_cb(self,msg):
        self.closest_node=msg.data
        
    def current_node_cb(self,msg):
        self.current_node=msg.data

    def get_nav_status_cb(self,msg):   
        self.nav_action_outcome=msg.status
    
    def main(self):


        # Wait for control-c
        rospy.spin()
        
        if rospy.is_shutdown():
            self.exp_times_handler.prism_client.shutdown(True)
            self.policy_handler.prism_client.shutdown(True)


if __name__ == '__main__':
    rospy.init_node('mdp_planner')
    
    if len(sys.argv)<2:
        print "usage: rosrun mdp_plan_exec mdp_planner <topological_map_name>"
        sys.exit(2)
        
    mdp_planner =  MdpPlanner(sys.argv[1])
    
    
    
    
    mdp_planner.main()
    
    
