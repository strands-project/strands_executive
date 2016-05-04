#! /usr/bin/env python
import os
import sys
import rospy

from mdp_plan_exec.top_map_mdp import TopMapMdp
from mdp_plan_exec.product_mdp import ProductMdp
from mdp_plan_exec.prism_java_talker import PrismJavaTalker

from std_msgs.msg import String
from actionlib import SimpleActionServer, SimpleActionClient
from actionlib_msgs.msg import GoalStatus

from strands_navigation_msgs.msg import NavRoute, ExecutePolicyModeAction, ExecutePolicyModeFeedback, ExecutePolicyModeGoal
from strands_executive_msgs.msg import ExecutePolicyAction, ExecutePolicyFeedback, ExecutePolicyGoal
from strands_executive_msgs.srv import GetSpecialWaypoints

   
class MdpPolicyExecutor(object):
    def __init__(self,top_map): 
        got_service=False
        while not got_service:
            try:
                rospy.wait_for_service('/mdp_plan_exec/get_special_waypoints', 1)
                got_service=True
            except rospy.ROSException,e:
                rospy.loginfo("Waiting for get_special_waypoints service...")
            if rospy.is_shutdown():
                return       
        self.special_waypoints_srv=rospy.ServiceProxy("/mdp_plan_exec/get_special_waypoints", GetSpecialWaypoints)
        self.special_waypoints=self.special_waypoints_srv()
        self.leaving_forbidden_waypoints=False
        
        self.top_nav_policy_exec= SimpleActionClient('/topological_navigation/execute_policy_mode', ExecutePolicyModeAction)
        got_server=self.top_nav_policy_exec.wait_for_server(rospy.Duration(1))
        while not got_server:
            rospy.loginfo("Waiting for topological navigation execute policy mode action server.")
            got_server=self.top_nav_policy_exec.wait_for_server(rospy.Duration(1))
            if rospy.is_shutdown():
                return
        
        self.top_map_mdp=TopMapMdp(top_map)
        self.directory = os.path.expanduser("~") + '/tmp/prism/policy_executor/'
        try:
            os.makedirs(self.directory)
        except OSError as ex:
            print 'error creating PRISM directory:',  ex
        self.file_name=top_map+".mdp"
        self.prism_policy_generator=PrismJavaTalker(8086,self.directory, self.file_name)
        
        self.current_prod_state=None
        self.product_mdp=None
        
        
        #self.learning_travel_times=False
        #self.learn_travel_times_action=SimpleActionServer('mdp_plan_exec/learn_travel_times', LearnTravelTimesAction, execute_cb = self.execute_learn_travel_times_cb, auto_start = False)
        #self.learn_travel_times_action.register_preempt_callback(self.preempt_learning_cb)
        #self.learn_travel_times_action.start()
               
        self.current_waypoint=None
        self.closest_waypoint=None
        self.current_waypoint_sub=rospy.Subscriber("/current_node", String, self.current_waypoint_cb)
        self.closest_waypoint_sub=rospy.Subscriber("/closest_node", String, self.closest_waypoint_cb)
        
        self.policy_mode_pub=rospy.Publisher("/mdp_plan_exec/current_policy_mode", NavRoute,queue_size=1)
        
        self.mdp_nav_as=SimpleActionServer('mdp_plan_exec/execute_policy', ExecutePolicyAction, execute_cb = self.execute_policy_cb, auto_start = False)
        self.mdp_nav_as.register_preempt_callback(self.preempt_policy_execution_cb)
        self.mdp_nav_as.start()
        
        rospy.loginfo("MDP policy executor initialised.")

    def current_waypoint_cb(self,msg):
        self.current_waypoint=msg.data
        
    def closest_waypoint_cb(self,msg):
        self.closest_waypoint=msg.data

    def generate_prism_specification(self, goal):
        self.special_waypoints=self.special_waypoints_srv()
        if goal.task_type==ExecutePolicyGoal.GOTO_WAYPOINT:
            if not self.top_map_mdp.target_in_topological_map(goal.target_id):
                rospy.logerr("Execute policy target  " + goal.target_id  + "  is not a node in the topological map. Aborting")
                return None
            if goal.target_id in self.special_waypoints.forbidden_waypoints:
                rospy.logwarn("The goal is a forbidden waypoint. Aborting")
                return None
            elif self.current_waypoint in self.special_waypoints.forbidden_waypoints:
                rospy.logwarn("The current position is forbidden. Aborting")
                return None
            if self.special_waypoints.forbidden_waypoints==[]:
                return 'R{"time"}min=? [ (F "' + goal.target_id + '") ]'
            else:
                return 'R{"time"}min=? [ (' + self.special_waypoints.forbidden_waypoints_ltl_string + ' U "' + goal.target_id + '") ]'
        elif goal.task_type==ExecutePolicyGoal.LEAVE_FORBIDDEN_AREA:
            if self.special_waypoints.forbidden_waypoints==[]:
                rospy.logwarn("No forbidden waypoints defined. Nothing to leave.")
                return None
            else:
                return 'R{"time"}min=? [ (F ' + self.special_waypoints.forbidden_waypoints_ltl_string + ') ]'
        elif goal.task_type==ExecutePolicyGoal.GOTO_CLOSEST_SAFE_WAYPOINT:
            if self.special_waypoints.safe_waypoints==[]:
                rospy.logwarn("No safe waypoints defined. Nowhere to go to.")
                return None
            else:
                return 'R{"time"}min=? [ (F ' + self.special_waypoints.safe_waypoints_ltl_string + ') ]'
        elif goal.task_type==ExecutePolicyGoal.COSAFE_LTL:
            return 'R{"time"}min=? [ (' + goal.target_id + ') ]'
    
    def generate_prod_mdp(self,specification):
        #update initial state
        self.top_map_mdp.set_initial_state_from_waypoint(self.closest_waypoint)
        self.top_map_mdp.add_predictions(self.directory+self.file_name,rospy.Time.now())
        expected_time=float(self.prism_policy_generator.get_policy(specification))
        feedback=ExecutePolicyFeedback(expected_time=expected_time)
        self.mdp_nav_as.publish_feedback(feedback)
        if feedback.expected_time==float("inf"):
            rospy.logwarn("The goal is unattainable. Aborting...")
            self.product_mdp=None
        else:
            self.product_mdp=ProductMdp(self.top_map_mdp,self.directory + '/prod.sta',self.directory + '/prod.lab',self.directory + '/prod.tra', self.directory+'prod.aut')
            self.product_mdp.write_prism_model(self.directory+'prod_'+self.file_name)
            self.product_mdp.set_policy(self.directory + '/adv.tra')  
        
    def generate_current_policy_mode(self, current_state):
        current_dra_state = current_state["dra_state1"]
        policy_msg = NavRoute()
        for state_def, action in zip(self.product_mdp.product_state_defs,self.product_mdp.policy):
            if action is not None and state_def["dra_state1"] == current_dra_state:
                source = self.top_map_mdp.get_waypoint_prop(state_def["waypoint"])
                policy_msg.source.append(source)
                policy_msg.edge_id.append(action)
        return policy_msg  

    def execute_policy_cb(self,goal):
        if goal.task_type==ExecutePolicyGoal.LEAVE_FORBIDDEN_AREA:
            self.leaving_forbidden_area=True
        else:
            self.leaving_forbidden_area=False
        specification=self.generate_prism_specification(goal)
        if specification is None:
            self.mdp_nav_as.set_aborted()
            return
        rospy.loginfo("The specification is: " + specification)
        self.generate_prod_mdp(specification)
        if self.product_mdp is None:
            self.mdp_nav_as.set_aborted()
            return
        self.current_prod_state=dict(self.product_mdp.initial_state)
        
        entered_exec_loop=False #needed to ensure that robot goes to exact pose when already in influence area of target
        while self.current_prod_state["dra_state1"] != self.product_mdp.props_def["dra_acc_state1"].conds["dra_state1"] or not entered_exec_loop:
            entered_exec_loop=True
            policy_mode_msg=self.generate_current_policy_mode(self.current_prod_state)
            self.policy_mode_pub.publish(policy_mode_msg)
            self.top_nav_policy_exec.send_goal(ExecutePolicyModeGoal(route = policy_mode_msg), feedback_cb = self.top_nav_feedback_cb)
            self.top_nav_policy_exec.wait_for_result()
            status=self.top_nav_policy_exec.get_state()  
            rospy.loginfo("Topological navigation execute policy action server exited with status: " + GoalStatus.to_string(status))
            if status!=GoalStatus.SUCCEEDED:
                if status==GoalStatus.ABORTED or self.current_prod_state is None:
                    self.mdp_nav_as.set_aborted()
                elif status==GoalStatus.PREEMPTED:
                    self.mdp_nav_as.set_preempted()
                else:
                    rospy.logwarn("Unexpected outcome from the topological navigaton execute policy action server. Setting as aborted")
                    self.mdp_nav_as.set_aborted()
                return
        rospy.loginfo("Policy execution successful.")
        self.mdp_nav_as.set_succeeded()


    def preempt_policy_execution_cb(self):     
        self.top_nav_policy_exec.cancel_all_goals()
    
    def top_nav_feedback_cb(self,feedback):
        rospy.loginfo("Reached waypoint " + feedback.route_status)
        if not self.leaving_forbidden_area and feedback.route_status in self.special_waypoints.forbidden_waypoints:
            rospy.logwarn("At forbidden waypoint!! Aborting")
            self.current_prod_state=None
            self.top_nav_policy_exec.cancel_all_goals()
        else:
            self.get_next_prod_state(feedback.route_status)
        
    def get_next_prod_state(self, current_waypoint):
        waypoint_val=self.product_mdp.props_def[current_waypoint].conds['waypoint']
        if waypoint_val==self.current_prod_state["waypoint"]:
            return
        for transition in self.product_mdp.transitions:
            if self.product_mdp.check_cond_sat(transition.pre_conds, self.current_prod_state):
                for prob_post_cond in transition.prob_post_conds:
                    if prob_post_cond[1]["waypoint"]==waypoint_val:
                        self.current_prod_state=dict(prob_post_cond[1])
                        return
        if self.current_prod_state["dra_state1"] == self.product_mdp.props_def["dra_acc_state1"].conds["dra_state1"]: 
            rospy.logwarn("Skipping MDP state update as target state has already been visited")
            return        
        rospy.logwarn("Error getting MDP next state: There is no transition modelling the state evolution. Keeping LTL automaton in the same state. Execution might have issues if the task is not simple reachability.")
        self.current_prod_state={'waypoint':waypoint_val,  'dra_state1':self.current_prod_state["dra_state1"]}

     
    def main(self):
        # Wait for control-c
        rospy.spin()       
        if rospy.is_shutdown():
            self.prism_policy_generator.shutdown(True)

if __name__ == '__main__':
    rospy.init_node('mdp_policy_executor')
    
    while not rospy.has_param("/topological_map_name") and not rospy.is_shutdown():
        rospy.sleep(0.1)

    if not rospy.is_shutdown():
        top_map_name=rospy.get_param("/topological_map_name")
        mdp_executor =  MdpPolicyExecutor(top_map_name)
        mdp_executor.main()
    
    
