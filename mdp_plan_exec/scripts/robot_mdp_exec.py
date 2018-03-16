#! /usr/bin/env python

import sys
import rospy

from mdp_plan_exec.robot_policy_executor import RobotPolicyExecutor



   

if __name__ == '__main__':
    rospy.init_node('robot_mdp_policy_executor')
    
    filtered_argv=rospy.myargv(argv=sys.argv)
    
    print "AHAH"
    
    if len(filtered_argv)!=4:
        rospy.logerr("Usage: rosrun mdp_plan_exec robot_mdp_exec.py port file_dir model_file")
    else:
        port = filtered_argv[1]
        file_dir= filtered_argv[2]
        model_file = filtered_argv[3] 

        mdp_executor =  RobotPolicyExecutor(int(port), file_dir, model_file)
        mdp_executor.main()
        
    
