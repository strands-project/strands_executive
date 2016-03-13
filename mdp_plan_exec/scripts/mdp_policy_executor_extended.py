#! /usr/bin/env python

import os
import sys
from copy import deepcopy
import rospy

from mdp_plan_exec.top_map_mdp import TopMapMdp
from mdp_plan_exec.policy_mdp import PolicyMdp
from mdp_plan_exec.prism_java_talker import PrismJavaTalker
from mdp_plan_exec.action_executor import ActionExecutor

from std_msgs.msg import String
from actionlib import SimpleActionServer, SimpleActionClient
from actionlib_msgs.msg import GoalStatus

from strands_navigation_msgs.msg import NavRoute, ExecutePolicyModeAction, ExecutePolicyModeFeedback, ExecutePolicyModeGoal
from strands_executive_msgs.msg import ExecutePolicyAction, ExecutePolicyFeedback, ExecutePolicyGoal
from strands_executive_msgs.msg import ExecutePolicyExtendedAction, ExecutePolicyExtendedFeedback, ExecutePolicyExtendedGoal

   
class MdpPolicyExecutor(object):
    def __init__(self,top_map): 

        self.wait_for_result_dur=rospy.Duration(0.1)
        self.top_nav_policy_exec= SimpleActionClient('/topological_navigation/execute_policy_mode', ExecutePolicyModeAction)
        got_server=self.top_nav_policy_exec.wait_for_server(rospy.Duration(1))
        while not got_server:
            rospy.loginfo("Waiting for topological navigation execute policy mode action server.")
            got_server=self.top_nav_policy_exec.wait_for_server(rospy.Duration(1))
            if rospy.is_shutdown():
                return
        
        self.top_map_mdp=TopMapMdp(top_map, explicit_doors=True)
        self.current_extended_mdp=None
        self.policy_mdp=None
        self.current_nav_policy_state_defs={}
        
        self.directory = os.path.expanduser("~") + '/tmp/prism/policy_executor_extended/'
        try:
            os.makedirs(self.directory)
        except OSError as ex:
            print 'error creating PRISM directory:',  ex
        self.file_name=top_map+".mdp"
        self.prism_policy_generator=PrismJavaTalker(8088,self.directory, self.file_name)
        
               
        self.current_waypoint=None
        self.closest_waypoint=None
        self.current_waypoint_sub=rospy.Subscriber("/current_node", String, self.current_waypoint_cb)
        self.closest_waypoint_sub=rospy.Subscriber("/closest_node", String, self.closest_waypoint_cb)       
        self.regenerate_policy=True
        self.policy_mode_pub=rospy.Publisher("/mdp_plan_exec/current_policy_mode", NavRoute,queue_size=1)
        
        self.action_executor=ActionExecutor()
        
        self.cancelled=False
        self.mdp_as=SimpleActionServer('mdp_plan_exec/execute_policy_extended', ExecutePolicyExtendedAction, execute_cb = self.execute_policy_cb, auto_start = False)
        self.mdp_as.register_preempt_callback(self.preempt_policy_execution_cb)
        self.mdp_as.start()
        

    def current_waypoint_cb(self,msg):
        self.current_waypoint=msg.data
        
    def closest_waypoint_cb(self,msg):
        self.closest_waypoint=msg.data

    def generate_prism_specification(self, ltl_spec):       
        return 'partial(R{"time"}min=? [ (' + ltl_spec + ') ])'
    
    def generate_policy_mdp(self,specification):
        #update initial state
        print(self.closest_waypoint)
        self.current_extended_mdp.set_initial_state_from_waypoint(self.closest_waypoint)
        self.current_extended_mdp.set_mdp_action_durations(self.directory+self.file_name,rospy.Time.now())
        expected_time=float(self.prism_policy_generator.get_policy(specification, partial_sat=True))
        #feedback=ExecutePolicyFeedback(expected_time=expected_time)
        #self.mdp_nav_as.publish_feedback(feedback)
        #if feedback.expected_time==float("inf"):
        if False:
            rospy.logwarn("The goal is unattainable. Aborting...")
            self.policy_mdp=None
        else:
            self.policy_mdp=PolicyMdp(self.current_extended_mdp,self.directory + '/prod.aut',self.directory + '/prod.sta',self.directory + '/prod.lab', self.directory+'adv.tra')
            self.current_policy_flat_state=self.policy_mdp.initial_flat_state
            

    def generate_current_nav_policy(self):
        policy_msg = NavRoute()
        current_state_def=self.policy_mdp.flat_state_defs[self.current_flat_state]
        current_dfa_state=current_state_def["_da"]
        queue=[self.current_flat_state]
        self.current_nav_policy_state_defs={self.current_flat_state:current_state_def}
        while queue != []:
            print(queue)
            new_flat_state=queue.pop(0)
            current_state_def=self.policy_mdp.flat_state_defs[new_flat_state]
            if self.policy_mdp.flat_state_policy.has_key(new_flat_state):
                action=self.policy_mdp.flat_state_policy[new_flat_state]
                if action in self.current_extended_mdp.nav_actions and current_state_def["_da"]==current_dfa_state:
                    source = self.current_extended_mdp.get_waypoint_prop(current_state_def["waypoint"])
                    policy_msg.source.append(source)
                    policy_msg.edge_id.append(action)
                    for suc_state in self.policy_mdp.flat_state_sucs[new_flat_state]:
                        if not self.current_nav_policy_state_defs.has_key(suc_state):
                            queue.append(suc_state)
                            self.current_nav_policy_state_defs[suc_state]=self.policy_mdp.flat_state_defs[suc_state]
        self.regenerate_policy=False
        self.policy_mode_pub.publish(policy_msg)
        return policy_msg
    
    
    def execute_nav_policy(self, nav_policy_msg):        
        self.top_nav_policy_exec.send_goal(ExecutePolicyModeGoal(route = nav_policy_msg), feedback_cb = self.top_nav_feedback_cb)
        nav_policy_finished=self.top_nav_policy_exec.wait_for_result(self.wait_for_result_dur)
        while not nav_policy_finished:
            if self.regenerate_policy:
                nav_policy_msg=self.generate_current_nav_policy()
                print(nav_policy_msg)
                self.top_nav_policy_exec.send_goal(ExecutePolicyModeGoal(route = nav_policy_msg), feedback_cb = self.top_nav_feedback_cb)
            nav_policy_finished=self.top_nav_policy_exec.wait_for_result(self.wait_for_result_dur)
        status=self.top_nav_policy_exec.get_state()  
        return status
    
    def top_nav_feedback_cb(self,feedback):
        rospy.loginfo("Reached waypoint " + feedback.route_status)
        self.get_next_policy_state(feedback.route_status)
        
    def get_next_policy_state(self, current_waypoint):
        waypoint_val=self.current_extended_mdp.get_waypoint_var_val(current_waypoint)
        current_state_def=self.current_nav_policy_state_defs[self.current_flat_state]
        #waypoint didnt change
        if waypoint_val==current_state_def["waypoint"]:
            return
        #waypoint change is on the MDP transition model
        for (flat_state, flat_state_def) in self.current_nav_policy_state_defs.items():
            print(flat_state_def)
            if flat_state_def["waypoint"]==waypoint_val:
                self.current_flat_state=flat_state
                return
        rospy.logwarn("Error getting MDP next state: There is no transition modelling the state evolution. Looking for state in full state list...")
        new_state_def=deepcopy(current_state_def)
        new_state_def["waypoint"]=waypoint_val
        for (flat_state, flat_state_def) in self.policy_mdp.flat_state_defs.items():
            if self.current_extended_mdp.check_cond_sat(new_state_def, flat_state_def):
                rospy.loginfo("Found state " + str(flat_state_def) + ". Updating.")
                self.current_flat_state=flat_state
                self.regenerate_policy=True #Assumes full policy export. In case we stop exporting full policy, change this so that replan is triggered when the new state doesnt have an action associated to it.
                return 
        rospy.logerr("Couldn't update state. Aborting...")
        self.current_flat_state=None
        self.top_nav_policy_exec.cancel_all_goals()
       
    def get_mdp_state_update_from_action_outcome(self, state_update):
        #current_state_def=self.current_nav_policy_state_defs[self.current_flat_state]
        current_state_def=self.policy_mdp.flat_state_defs[self.current_flat_state]
        current_state_def.update(state_update)
        del current_state_def["_da"]
        for flat_suc in self.policy_mdp.flat_state_sucs[self.current_flat_state]:
            if self.policy_mdp.check_cond_sat(current_state_def, self.policy_mdp.flat_state_defs[flat_suc]):
                self.current_flat_state=flat_suc
                return
        rospy.logerr("FODEU")


    def execute_policy_cb(self,goal):
        specification=self.generate_prism_specification(goal.spec.ltl_task)
        self.current_extended_mdp=deepcopy(self.top_map_mdp)
        self.current_extended_mdp.add_extra_domain(goal.spec.vars, goal.spec.actions)
        
        rospy.loginfo("The specification is: " + specification)
        self.generate_policy_mdp(specification)
        if self.policy_mdp is None:
            self.mdp_as.set_aborted()
            return
        self.current_flat_state=self.policy_mdp.initial_flat_state
        
        current_nav_policy=self.generate_current_nav_policy()
        status=self.execute_nav_policy(current_nav_policy)
        while self.policy_mdp.flat_state_policy.has_key(self.current_flat_state) and not self.cancelled:
            next_action=self.policy_mdp.flat_state_policy[self.current_flat_state]
            if next_action in self.current_extended_mdp.nav_actions:
                current_nav_policy=self.generate_current_nav_policy()
                status=self.execute_nav_policy(current_nav_policy)
                rospy.loginfo("Topological navigation execute policy action server exited with status: " + GoalStatus.to_string(status))
                if status!=GoalStatus.SUCCEEDED:
                    if status==GoalStatus.ABORTED or self.current_flat_state is None:
                        self.mdp_as.set_aborted()
                    elif status==GoalStatus.PREEMPTED:
                        self.mdp_as.set_preempted()
                    else:
                        rospy.logwarn("Unexpected outcome from the topological navigaton execute policy action server. Setting as aborted")
                        self.mdp_as.set_aborted()
                    return
            else:
                print("EXECUTE ACTION")
                starting_waypoint=self.current_waypoint
                (status, state_update)=self.action_executor.execute_action(self.current_extended_mdp.action_descriptions[next_action])
                if not self.cancelled:
                    self.mdp_as.publish_feedback(ExecutePolicyExtendedFeedback(starting_waypoint=starting_waypoint,
                                                                               executed_action=self.current_extended_mdp.action_descriptions[next_action].action_server,
                                                                               execution_status=status))
                    self.get_mdp_state_update_from_action_outcome(state_update)
        
        
        if self.cancelled:
            self.cancelled=False
            self.mdp_as.set_preempted()
        else:
            rospy.loginfo("Policy execution successful.")
            self.mdp_as.set_succeeded()


    def preempt_policy_execution_cb(self):     
        self.top_nav_policy_exec.cancel_all_goals()
        self.action_executor.cancel_all_goals()
        self.cancelled=True

     
    def main(self):
        # Wait for control-c
        rospy.spin()       
        if rospy.is_shutdown():
            self.prism_policy_generator.shutdown(True)

if __name__ == '__main__':
    rospy.init_node('mdp_policy_executor_extended')
    top_map_name=rospy.get_param("/topological_map_name")
    mdp_executor =  MdpPolicyExecutor(top_map_name)
    mdp_executor.main()
    
    
