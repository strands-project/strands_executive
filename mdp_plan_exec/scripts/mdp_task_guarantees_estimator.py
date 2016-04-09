#! /usr/bin/env python

import sys
import os
from copy import deepcopy
import rospy

from mdp_plan_exec.top_map_mdp import TopMapMdp
from mdp_plan_exec.prism_java_talker import PrismJavaTalker

from strands_executive_msgs.srv import GetGuaranteesForCoSafeTask, GetGuaranteesForCoSafeTaskResponse


class MdpTaskGuaranteesEstimator(object):

    def __init__(self,top_map):
        
        self.top_map_mdp=TopMapMdp(top_map, explicit_doors=True)
        self.current_extended_mdp=None
        self.directory = os.path.expanduser("~") + '/tmp/prism/guarantees_estimator/'
        try:
            os.makedirs(self.directory)
        except OSError as ex:
            print 'error creating PRISM directory:',  ex
        self.file_name=top_map+".mdp"
        self.prism_estimator=PrismJavaTalker(8087,self.directory, self.file_name)
        self.last_epoch=-1
        self.get_guarantees_service = rospy.Service('/mdp_plan_exec/get_guarantees_for_co_safe_task',
                                                              GetGuaranteesForCoSafeTask,
                                                              self.get_guarantees_cb)
        rospy.loginfo("MDP task guarantees estimator initialised.")

    def generate_prism_specification(self, ltl_spec):       
        return 'partial(R{"time"}min=? [ (' + ltl_spec + ') ])'

    def get_guarantees_cb(self,req):
        self.current_extended_mdp=deepcopy(self.top_map_mdp)
        self.current_extended_mdp.add_extra_domain(req.spec.vars, req.spec.actions)
        if req.epoch != self.last_epoch:
            self.last_epoch=req.epoch
        self.current_extended_mdp.set_mdp_action_durations(self.directory+self.file_name, req.epoch, set_initial_state=False) #TODO we need this here to update the MDP model file, but we shouldn't need to get new action durations to do it when the epoch didnt change.
        specification=self.generate_prism_specification(req.spec.ltl_task)
        rospy.loginfo("The specification is " + specification)
        state_vector=self.prism_estimator.get_state_vector(specification, is_partial=True)
        (waypoint_names, waypoint_probs, waypoint_progs, waypoint_times)=self.current_extended_mdp.map_state_vectors_to_waypoint_expectations(self.directory+'original.sta', state_vector)
        return GetGuaranteesForCoSafeTaskResponse(initial_waypoints=waypoint_names,
                                                  probabilities=waypoint_probs,
                                                  prog_rewards=waypoint_progs,
                                                  expected_times=map(rospy.Duration, waypoint_times))
        

    def main(self):
       # Wait for control-c
        rospy.spin()       
        if rospy.is_shutdown():
            self.prism_estimator.shutdown(True)


if __name__ == '__main__':
    rospy.init_node('mdp_task_guarantees_estimator')
    
    while not rospy.has_param("/topological_map_name") and not rospy.is_shutdown():
        rospy.sleep(0.1)

    if not rospy.is_shutdown():
        top_map_name=rospy.get_param("/topological_map_name")
        mdp_estimator =  MdpTaskGuaranteesEstimator(top_map_name)
        mdp_estimator.main()
    
    
