#! /usr/bin/env python

import os
import sys
from copy import deepcopy
import rospy

from strands_navigation_msgs.msg import NavRoute #TODO: remove this ependency and create a nav route that is not tied to this msg definition. 

from mdp_plan_exec.top_map_mdp import TopMapMdp
from mdp_plan_exec.policy_mdp import PolicyMdp
from mdp_plan_exec.partial_sat_prism_java_talker import PartialSatPrismJavaTalker

from std_msgs.msg import String
from actionlib import SimpleActionServer, SimpleActionClient
from actionlib_msgs.msg import GoalStatus

from strands_executive_msgs.msg import ExecutePolicyAction, ExecutePolicyFeedback, ExecutePolicyGoal
from strands_executive_msgs.msg import ExecutePolicyExtendedAction, ExecutePolicyExtendedFeedback, ExecutePolicyExtendedGoal

   
class MdpPolicyExecutor(object):
    def __init__(self, port, file_dir, file_name): 
        
        
        self.mdp=TopMapMdp(explicit_doors=True, forget_doors=True, model_fatal_fails=True)
        self.policy_mdp=None
        self.current_nav_policy_state_defs={}
        
        self.directory = file_dir
        self.file_name = file_name
        try:
            os.makedirs(self.directory)
        except OSError as ex:
            print 'error creating PRISM directory:',  ex
        self.prism_policy_generator=PartialSatPrismJavaTalker(port,self.directory, self.file_name)
        
               
        self.current_waypoint=None
        self.closest_waypoint=None
   
        self.regenerate_policy=True

        
        self.cancelled=False


    def generate_prism_specification(self, ltl_spec):       
        return 'partial(R{"time"}min=? [ (' + ltl_spec + ') ])'
    
    def get_current_action(self):
        if self.policy_mdp.flat_state_policy.has_key(self.current_flat_state):
            return self.policy_mdp.flat_state_policy[self.current_flat_state]
        else:
            return ''
    
    # Returns True iff the policy was generated correctly
    def generate_policy_mdp(self, spec):
        specification=self.generate_prism_specification(spec.ltl_task)
        self.mdp.create_top_map_mdp_structure()
        self.mdp.add_extra_domain(spec.vars, spec.actions)
        
        rospy.loginfo("The specification is: " + specification)
    
        #update initial state
        self.mdp.set_initial_state_from_waypoint(self.closest_waypoint)
        self.mdp.add_predictions(self.directory+self.file_name,rospy.Time.now())
        prism_call_success=self.prism_policy_generator.call_prism(specification)
        if prism_call_success:
            self.policy_mdp=PolicyMdp(self.mdp,
                                      self.directory + 'prod.sta',
                                      self.directory + 'prod.lab',
                                      self.directory + 'adv.tra',
                                      self.directory + 'guarantees1.vect',
                                      self.directory + 'guarantees2.vect',
                                      self.directory + 'guarantees3.vect'
                                      )
            self.current_policy_flat_state=self.policy_mdp.initial_flat_state
            return True
        else:
            rospy.logwarn("Error generating policy. Aborting...")
            self.policy_mdp=None
            return False
            

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
                if action in self.mdp.nav_actions and current_state_def["_da"]==current_dfa_state:
                    source = self.mdp.get_waypoint_prop(current_state_def["waypoint"])
                    policy_msg.source.append(source)
                    policy_msg.edge_id.append(action)
                    for suc_state in self.policy_mdp.flat_state_sucs[new_flat_state]:
                        if not self.current_nav_policy_state_defs.has_key(suc_state):
                            queue.append(suc_state)
                            self.current_nav_policy_state_defs[suc_state]=self.policy_mdp.flat_state_defs[suc_state]
        self.regenerate_policy=False
        return policy_msg
    
    def get_next_nav_policy_state(self, current_waypoint, nav_action_status):
        publish = True
        executed_action=self.get_current_action()
        next_action = None
        waypoint_val=self.mdp.get_waypoint_var_val(current_waypoint)
        found_next_state = False
        #if got feedback from topo nav in a waypoint, check if need to update state
        if waypoint_val > -1 and nav_action_status == GoalStatus.SUCCEEDED:
            current_state_def=self.current_nav_policy_state_defs[self.current_flat_state]
            #only update state if waypoint changed
            if waypoint_val==current_state_def["waypoint"]:
                publish = False
            else:
                rospy.loginfo("Reached waypoint " + current_waypoint)
                #update state
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
                        if self.mdp.check_cond_sat(new_state_def, flat_state_def):
                            rospy.loginfo("Found state " + str(flat_state_def) + ". Updating.")
                            self.current_flat_state=flat_state
                            found_next_state=True
                            self.regenerate_policy=True #Assumes full policy export. In case we stop exporting full policy, change this so that replan is triggered when the new state doesnt have an action associated to it.
                            break
        if publish:
            self.publish_feedback(executed_action, nav_action_status, next_action)

        if found_next_state:
            next_action=self.get_current_action()
            return True
        else:
            self.current_flat_state=None
            return False
            
        
       
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

