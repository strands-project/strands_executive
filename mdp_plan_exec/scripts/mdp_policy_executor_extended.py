#! /usr/bin/env python

import os
import sys
from copy import deepcopy
import rospy

from mdp_plan_exec.top_map_mdp import TopMapMdp
from mdp_plan_exec.policy_mdp import PolicyMdp
from mdp_plan_exec.partial_sat_prism_java_talker import PartialSatPrismJavaTalker
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
        
        self.top_map_mdp=TopMapMdp(top_map, explicit_doors=True, forget_doors=True, model_fatal_fails=True)
        self.current_extended_mdp=None
        self.policy_mdp=None
        self.current_nav_policy_state_defs={}
        
        self.directory = os.path.expanduser("~") + '/tmp/prism/policy_executor_extended/'
        try:
            os.makedirs(self.directory)
        except OSError as ex:
            print 'error creating PRISM directory:',  ex
        self.file_name=top_map+".mdp"
        self.prism_policy_generator=PartialSatPrismJavaTalker(8088,self.directory, self.file_name)
        
               
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
    
    def get_current_action(self):
        if self.policy_mdp.flat_state_policy.has_key(self.current_flat_state):
            return self.policy_mdp.flat_state_policy[self.current_flat_state]
        else:
            return ''
    
    def generate_policy_mdp(self,specification):
        #update initial state
        self.current_extended_mdp.set_initial_state_from_waypoint(self.closest_waypoint)
        self.current_extended_mdp.add_predictions(self.directory+self.file_name,rospy.Time.now())
        prism_call_success=self.prism_policy_generator.call_prism(specification)
        if prism_call_success:
            self.policy_mdp=PolicyMdp(self.current_extended_mdp,
                                      self.directory + 'prod.sta',
                                      self.directory + 'prod.lab',
                                      self.directory + 'adv.tra',
                                      self.directory + 'guarantees1.vect',
                                      self.directory + 'guarantees2.vect',
                                      self.directory + 'guarantees3.vect'
                                      )
            self.current_policy_flat_state=self.policy_mdp.initial_flat_state
        else:
            rospy.logwarn("Error generating policy. Aborting...")
            self.policy_mdp=None
            

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
        self.get_next_nav_policy_state(feedback.route_status)
        
    def get_next_nav_policy_state(self, current_waypoint):
        executed_action=self.get_current_action()
        waypoint_val=self.current_extended_mdp.get_waypoint_var_val(current_waypoint)
        current_state_def=self.current_nav_policy_state_defs[self.current_flat_state]
        
        #waypoint didnt change
        if waypoint_val==current_state_def["waypoint"]:
            return
        
        found_next_state=False
        
        #waypoint change is on the MDP transition model
        for (flat_state, flat_state_def) in self.current_nav_policy_state_defs.items():
            print(flat_state_def)           
            if flat_state_def["waypoint"]==waypoint_val:
                self.current_flat_state=flat_state
                found_next_state=True
                break
                
        if not found_next_state: 
            rospy.logwarn("Error getting MDP next state: There is no transition modelling the state evolution. Looking for state in full state list...")
            new_state_def=deepcopy(current_state_def)
            new_state_def["waypoint"]=waypoint_val
            for (flat_state, flat_state_def) in self.policy_mdp.flat_state_defs.items():
                if self.current_extended_mdp.check_cond_sat(new_state_def, flat_state_def):
                    rospy.loginfo("Found state " + str(flat_state_def) + ". Updating.")
                    self.current_flat_state=flat_state
                    found_next_state=True
                    self.regenerate_policy=True #Assumes full policy export. In case we stop exporting full policy, change this so that replan is triggered when the new state doesnt have an action associated to it.
                    break
            
        if found_next_state:    
            next_action=self.get_current_action()
            if self.policy_mdp.flat_state_defs[self.current_flat_state]['waypoint']==-1:
                outcome=GoalStatus.ABORTED
            else:
                outcome=GoalStatus.SUCCEEDED
            self.publish_feedback(executed_action, outcome, next_action)
        else:
            rospy.logerr("Couldn't update state. Aborting...")
            self.current_flat_state=None
            self.top_nav_policy_exec.cancel_all_goals()
       
    def get_mdp_state_update_from_action_outcome(self, state_update):
        #current_state_def=self.current_nav_policy_state_defs[self.current_flat_state]
        current_state_def=deepcopy(self.policy_mdp.flat_state_defs[self.current_flat_state])
        current_state_def.update(state_update)
        del current_state_def["_da"]
        for flat_suc in self.policy_mdp.flat_state_sucs[self.current_flat_state]:
            if self.policy_mdp.check_cond_sat(current_state_def, self.policy_mdp.flat_state_defs[flat_suc]):
                self.current_flat_state=flat_suc
                return
        rospy.logerr("Error finding next state after action execution. Aborting...")
        self.current_flat_state=None


    def execute_policy_cb(self,goal):
        specification=self.generate_prism_specification(goal.spec.ltl_task)
        self.current_extended_mdp=deepcopy(self.top_map_mdp)
        self.current_extended_mdp.add_extra_domain(goal.spec.vars, goal.spec.actions)
        
        rospy.loginfo("The specification is: " + specification)
        self.generate_policy_mdp(specification)
        if self.policy_mdp is None:
            print 'ABORTING HERE'
            self.mdp_as.set_aborted()
            return
        self.current_flat_state=self.policy_mdp.initial_flat_state
        self.publish_feedback(None, None, self.get_current_action())
        
        current_nav_policy=self.generate_current_nav_policy()
        status=self.execute_nav_policy(current_nav_policy)
        while self.policy_mdp.flat_state_policy.has_key(self.current_flat_state) and not self.cancelled:
            next_action=self.get_current_action()
            if next_action in self.current_extended_mdp.nav_actions:
                current_nav_policy=self.generate_current_nav_policy()
                status=self.execute_nav_policy(current_nav_policy)
                rospy.loginfo("Topological navigation execute policy action server exited with status: " + GoalStatus.to_string(status))                
                if status!=GoalStatus.SUCCEEDED:

                    # If mdp exec was preempted, this may cause the top nav to be preempted
                    # Therefore we need to make sure that the cancellation flag is switched off 
                    # as it won't reach line 218 where it is otherwise reset.
                    self.cancelled=False

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
                (status, state_update)=self.action_executor.execute_action(self.current_extended_mdp.action_descriptions[next_action])
                executed_action=next_action
                print(executed_action)
                if not self.cancelled:
                    self.get_mdp_state_update_from_action_outcome(state_update)
                    if self.current_flat_state is None:
                        self.mdp_as.set_aborted()
                        return
                    next_action=self.get_current_action()
                    self.publish_feedback(executed_action, status, next_action)
                else:
                    break

        
        
        if self.cancelled:
            self.cancelled=False
            rospy.loginfo("Policy execution preempted.")
            self.mdp_as.set_preempted()
        else:
            rospy.loginfo("Policy execution successful.")
            self.mdp_as.set_succeeded()

    def publish_feedback(self, executed_action, status, next_action):
        (probability, prog_reward, expected_time)=self.policy_mdp.get_guarantees_at_flat_state(self.current_flat_state)
        self.mdp_as.publish_feedback(ExecutePolicyExtendedFeedback(probability=probability,
                                                                expected_time=expected_time,
                                                                prog_reward=prog_reward,
                                                                current_waypoint=self.current_waypoint,
                                                                executed_action=executed_action,
                                                                execution_status=status,
                                                                next_action=next_action))

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

    while not rospy.has_param("/topological_map_name") and not rospy.is_shutdown():
        rospy.sleep(0.1)

    if not rospy.is_shutdown():
        top_map_name=rospy.get_param("/topological_map_name")
        mdp_executor =  MdpPolicyExecutor(top_map_name)
        mdp_executor.main()
        
    
