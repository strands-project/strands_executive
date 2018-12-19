#! /usr/bin/env python

import sys
import rospy
import threading
from mdp_plan_exec.top_map_mdp import TopMapMdp
from mdp_plan_exec.policy_execution_utils import PolicyExecutionUtils

from strands_executive_msgs.srv import GetGuaranteesForCoSafeTask, GetGuaranteesForCoSafeTaskResponse

class MdpTaskGuaranteesEstimator(object):

    def __init__(self, port, file_dir, file_name):
        explicit_doors = rospy.get_param("mdp_plan_exec/explicit_doors", True)
        forget_doors = rospy.get_param("mdp_plan_exec/forget_doors", True)
        model_fatal_fails = rospy.get_param("mdp_plan_exec/model_fatal_fails", True)

        mdp=TopMapMdp(explicit_doors=explicit_doors, forget_doors=forget_doors, model_fatal_fails=model_fatal_fails)
        self.policy_utils = PolicyExecutionUtils(port, file_dir, file_name, mdp)

        self.service_lock = threading.Lock()
        self.get_guarantees_service = rospy.Service('mdp_plan_exec/get_guarantees_for_co_safe_task', GetGuaranteesForCoSafeTask, self.get_guarantees_cb)
        rospy.loginfo("MDP task guarantees estimator initialised.")

    def get_guarantees_cb(self, req):
        with self.service_lock:
            response=GetGuaranteesForCoSafeTaskResponse()
            policy_mdp = self.policy_utils.generate_policy_mdp(req.spec, req.initial_waypoint, req.epoch)
            if policy_mdp is None:
                rospy.logwarn("Error calling PRISM, guarantees extimator service returning empty response")
            else:
                (response.probability, response.prog_reward, response.expected_time) = policy_mdp.get_guarantees_at_flat_state(policy_mdp.initial_flat_state)
            return response

    def main(self):
       # Wait for control-c
        rospy.spin()
        if rospy.is_shutdown():
            self.policy_utils.shutdown_prism(True)


if __name__ == '__main__':
    rospy.init_node('mdp_task_guarantees_estimator')
    
    filtered_argv=rospy.myargv(argv=sys.argv)
    
    if len(filtered_argv)<4:
        rospy.logerr("Usage: rosrun mdp_plan_exec mdp_task_guarantees_estimator port file_dir model_file")
    else:
        
        port = filtered_argv[1]
        file_dir = filtered_argv[2]
        model_file = filtered_argv[3] 

        mdp_estimator = MdpTaskGuaranteesEstimator(int(port), file_dir, model_file)
        mdp_estimator.main()
