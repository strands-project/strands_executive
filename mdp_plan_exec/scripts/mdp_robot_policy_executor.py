#! /usr/bin/env python

import sys
import rospy

from strands_navigation_msgs.msg import NavRoute, ExecutePolicyModeAction, ExecutePolicyModeFeedback, ExecutePolicyModeGoal

from mdp_plan_exec.top_map_mdp import TopMapMdp
from mdp_plan_exec.action_executor import ActionExecutor
from mdp_plan_exec.policy_execution_utils import PolicyExecutionUtils

from std_msgs.msg import String
from actionlib import SimpleActionServer, SimpleActionClient
from actionlib_msgs.msg import GoalStatus

from strands_executive_msgs.msg import ExecutePolicyAction, ExecutePolicyFeedback, ExecutePolicyGoal
from strands_executive_msgs.msg import ExecutePolicyExtendedAction, ExecutePolicyExtendedFeedback, ExecutePolicyExtendedGoal

   
class RobotPolicyExecutor():
    def __init__(self, port, file_dir, file_name): 
        
        
        self.wait_for_result_dur=rospy.Duration(0.1)
        self.top_nav_policy_exec= SimpleActionClient('topological_navigation/execute_policy_mode', ExecutePolicyModeAction)
        got_server=self.top_nav_policy_exec.wait_for_server(rospy.Duration(1))
        while not got_server:
            rospy.loginfo("Waiting for topological navigation execute policy mode action server.")
            got_server=self.top_nav_policy_exec.wait_for_server(rospy.Duration(1))
            if rospy.is_shutdown():
                return
        self.policy_mode_pub=rospy.Publisher("mdp_plan_exec/current_policy_mode", NavRoute,queue_size=1)

        self.current_waypoint_sub=rospy.Subscriber("current_node", String, self.current_waypoint_cb)
        self.closest_waypoint_sub=rospy.Subscriber("closest_node", String, self.closest_waypoint_cb)       

        self.mdp=TopMapMdp(explicit_doors=True, forget_doors=True, model_fatal_fails=True)
        self.policy_utils = PolicyExecutionUtils(port, file_dir, file_name, self.mdp)
        self.action_executor=ActionExecutor()
    
        self.cancelled=False
        self.mdp_as=SimpleActionServer('mdp_plan_exec/execute_policy_extended', ExecutePolicyExtendedAction, execute_cb = self.execute_policy_cb, auto_start = False)
        self.mdp_as.register_preempt_callback(self.preempt_policy_execution_cb)
        self.mdp_as.start()
        

    def current_waypoint_cb(self,msg):
        self.current_waypoint=msg.data
        
    def closest_waypoint_cb(self,msg):
        self.closest_waypoint=msg.data

    def execute_nav_policy(self, nav_policy_msg):
        self.policy_mode_pub.publish(nav_policy_msg)
        self.top_nav_policy_exec.send_goal(ExecutePolicyModeGoal(route = nav_policy_msg), feedback_cb = self.top_nav_feedback_cb)
        nav_policy_finished=self.top_nav_policy_exec.wait_for_result(self.wait_for_result_dur)
        while not nav_policy_finished:
            #if self.regenerate_policy: TODO: Figure out where to put this
                #nav_policy_msg=self.policy_utils.generate_current_nav_policy()
                #print(nav_policy_msg)
                #self.top_nav_policy_exec.send_goal(ExecutePolicyModeGoal(route = nav_policy_msg), feedback_cb = self.top_nav_feedback_cb)
            nav_policy_finished=self.top_nav_policy_exec.wait_for_result(self.wait_for_result_dur)
        status=self.top_nav_policy_exec.get_state()
        return status
    
    
    def top_nav_feedback_cb(self, feedback):
        executed_action=self.policy_mdp.get_current_action()
        next_flat_state = self.policy_utils.get_next_nav_policy_state(feedback.current_wp, feedback.status, self.policy_mdp)
        publish = next_flat_state != self.policy_mdp.current_flat_state
        self.policy_mdp.set_current_state(next_flat_state)
        if next_flat_state is None:
            rospy.logerr("Couldn't update state. Aborting...") #TODO:make sure action server returns aborted when this happens
            self.top_nav_policy_exec.cancel_all_goals()
        if publish:
            next_action=self.policy_mdp.get_current_action()
            self.publish_feedback(executed_action, feedback.status, next_action)



    def execute_policy_cb(self, goal):
        self.cancelled=False
        self.policy_mdp = self.policy_utils.generate_policy_mdp(goal.spec, self.closest_waypoint)
        
        if self.policy_mdp is None:
            print "OH NO" #TODO abort action server properly
        else:
            self.publish_feedback(None, None, self.policy_mdp.get_current_action())
            
            starting_exec = True #used to make sure the robot executes calls topological navigation at least once before executing non-nav actions. This is too ensure the robot navigates to the exact pose of a waypoint before executing an action there
            
            while (self.policy_mdp.has_action_defined() and not self.cancelled) or starting_exec:
                next_action=self.policy_mdp.get_current_action()
                if next_action in self.mdp.nav_actions or starting_exec:
                    starting_exec = False
                    current_nav_policy=self.policy_utils.generate_current_nav_policy(self.policy_mdp)
                    status=self.execute_nav_policy(current_nav_policy)
                    rospy.loginfo("Topological navigation execute policy action server exited with status: " + GoalStatus.to_string(status))
                    if status!=GoalStatus.SUCCEEDED:

                        # If mdp exec was preempted, this may cause the top nav to be preempted
                        # Therefore we need to make sure that the cancellation flag is switched off 
                        # as it won't reach line 218 where it is otherwise reset.
                        self.cancelled=False

                        if status==GoalStatus.PREEMPTED:
                            self.mdp_as.set_preempted()
                        elif status==GoalStatus.ABORTED or self.policy_mdp.current_flat_state is None:
                            self.mdp_as.set_aborted()
                        else:
                            rospy.logwarn("Unexpected outcome from the topological navigaton execute policy action server. Setting as aborted")
                            self.mdp_as.set_aborted()
                        return
                else:
                    print("EXECUTE ACTION")
                    (status, state_update)=self.action_executor.execute_action(self.mdp.action_descriptions[next_action])
                    executed_action=next_action
                    print(executed_action)
                    if not self.cancelled:
                        next_flat_state = self.policy_utils.get_next_state_from_action_outcome(state_update, self.policy_mdp)
                        policy_mdp.set_current_state(next_flat_state)
                        if next_flat_state is None:
                            rospy.logerr("Error finding next state after action execution. Aborting...")
                            self.mdp_as.set_aborted()
                        next_action=self.policy_mdp.get_current_action()
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
        (probability, prog_reward, expected_time)=self.policy_mdp.get_guarantees_at_current_state()
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
            self.policy_utils.shutdown_prism(True)


if __name__ == '__main__':
    rospy.init_node('robot_mdp_policy_executor')
    
    filtered_argv=rospy.myargv(argv=sys.argv)
    
    
    if len(filtered_argv)!=4:
        rospy.logerr("Usage: rosrun mdp_plan_exec robot_mdp_exec.py port file_dir model_file")
    else:
        port = filtered_argv[1]
        file_dir= filtered_argv[2]
        model_file = filtered_argv[3] 

        mdp_executor =  RobotPolicyExecutor(int(port), file_dir, model_file)
        mdp_executor.main()
        
    

