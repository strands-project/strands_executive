#! /usr/bin/env python

from __future__ import with_statement 
import sys
import os
from copy import deepcopy
import rospy
import threading
from mdp_plan_exec.top_map_mdp import TopMapMdp
from mdp_plan_exec.policy_mdp import PolicyMdp
from mdp_plan_exec.partial_sat_prism_java_talker import PartialSatPrismJavaTalker

from strands_executive_msgs.srv import GetGuaranteesForCoSafeTask, GetGuaranteesForCoSafeTaskResponse
from random import choice

class MdpTaskGuaranteesEstimator(object):

    def __init__(self):
        
        self.top_map_mdp=TopMapMdp(explicit_doors=True, forget_doors=True, model_fatal_fails=True)
        self.current_extended_mdp=None
        self.policy_mdp=None
        self.directory = os.path.expanduser("~") + '/tmp/prism/guarantees_estimator/'
        self.service_lock = threading.Lock()
        try:
            os.makedirs(self.directory)
        except OSError as ex:
            print 'error creating PRISM directory:',  ex
        self.file_name="topo_map.mdp"
        self.prism_estimator=PartialSatPrismJavaTalker(8087,self.directory, self.file_name)
        self.get_guarantees_service = rospy.Service('/mdp_plan_exec/get_guarantees_for_co_safe_task',
                                                              GetGuaranteesForCoSafeTask,
                                                              self.get_guarantees_cb)
        rospy.loginfo("MDP task guarantees estimator initialised.")

    def generate_prism_specification(self, ltl_spec):       
        return 'partial(R{"time"}min=? [ (' + ltl_spec + ') ])'

    def get_guarantees_cb(self,req):
        with self.service_lock:
            response=GetGuaranteesForCoSafeTaskResponse()
            self.top_map_mdp.create_top_map_mdp_structure()
            self.current_extended_mdp=deepcopy(self.top_map_mdp)
            self.current_extended_mdp.set_initial_state_from_waypoint(req.initial_waypoint)
            self.current_extended_mdp.add_extra_domain(req.spec.vars, req.spec.actions)
            self.current_extended_mdp.add_predictions(self.directory+self.file_name, req.epoch, set_initial_state=True) #add epoch - guarantees dict?
            specification=self.generate_prism_specification(req.spec.ltl_task)
            rospy.loginfo("The specification is " + specification)
            prism_call_success=self.prism_estimator.call_prism(specification)
            if prism_call_success:
                self.policy_mdp=PolicyMdp(self.current_extended_mdp,
                                          self.directory + '/prod.sta',
                                          self.directory + '/prod.lab',
                                          self.directory+'adv.tra',
                                          self.directory+'guarantees1.vect',
                                          self.directory+'guarantees2.vect',
                                          self.directory+'guarantees3.vect')
                (response.probability, response.prog_reward, response.expected_time)=self.policy_mdp.get_guarantees_at_flat_state(self.policy_mdp.initial_flat_state)
                self.add_plan(self.policy_mdp, response, req.epoch)
                return response
            else:
                rospy.logwarn("Error calling PRISM, guarantees extimator service returning empty response")
                return response
    
    def get_waypoint(self, state_def):
        return self.current_extended_mdp.get_waypoint_prop(state_def["waypoint"])
    
    def check_closed_doors(self, state_def):
        for (state_var, state_val) in state_def.iteritems():
            if 'door' in state_var:
                if state_val == 0:
                    return True
        return False
    
    def add_plan(self, policy_mdp, response, epoch):
        current_flat_state = policy_mdp.initial_flat_state
        current_state_def=policy_mdp.flat_state_defs[current_flat_state]
        plan = [self.get_waypoint(current_state_def)]
        durations = []
        predictions=self.top_map_mdp.get_edge_estimates(epoch)
        while True:
            wp = self.get_waypoint(current_state_def)
            if plan[-1] != wp:
                plan.append(wp)
                duration = predictions.durations[predictions.edge_ids.index(action)]
                durations.append(duration)
            if not policy_mdp.flat_state_policy.has_key(current_flat_state):
                break
            action = self.policy_mdp.flat_state_policy[current_flat_state]
            flat_succs = policy_mdp.flat_state_sucs[current_flat_state]
            for flat_succ in flat_succs:
                succ_def = policy_mdp.flat_state_defs[flat_succ]
                if succ_def['waypoint'] != -1 and not self.check_closed_doors(succ_def):
                    current_flat_state = flat_succ
                    current_state_def = policy_mdp.flat_state_defs[flat_succ]
                    break

        response.plan = plan
        response.durations = durations
            
            
       

    def main(self):
       # Wait for control-c
        rospy.spin()       
        if rospy.is_shutdown():
            self.prism_estimator.shutdown(True)


if __name__ == '__main__':
    rospy.init_node('mdp_task_guarantees_estimator')

    mdp_estimator =  MdpTaskGuaranteesEstimator()
    mdp_estimator.main()
    
    
