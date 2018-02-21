#! /usr/bin/env python

import rospy
from mongodb_store.message_store import MessageStoreProxy
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from strands_executive_msgs.msg import ExecutePolicyExtendedActionFeedback, ExecutePolicyExtendedActionGoal
from strands_executive_msgs.msg import TaskExecutionStat, ActionExecutionStat

   
class Logger(object):
    def __init__(self):
        
        self.msp = MessageStoreProxy(collection='mdp_exec_logs')
        self.active_goals = {}
        self.terminal_states= [GoalStatus.PREEMPTED, GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.LOST]
        
        mdp_goal_sub = rospy.Subscriber("mdp_plan_exec/execute_policy_extended/goal", ExecutePolicyExtendedActionGoal, self.mdp_goal_cb)
        mdp_feedback_sub = rospy.Subscriber("mdp_plan_exec/execute_policy_extended/feedback", ExecutePolicyExtendedActionFeedback, self.mdp_feedback_cb)
        mdp_status_sub = rospy.Subscriber("mdp_plan_exec/execute_policy_extended/status", GoalStatusArray, self.mdp_status_cb)

    def mdp_feedback_cb(self, msg):
        goal_id = msg.status.goal_id.id
        if self.active_goals.has_key(goal_id):
            self.active_goals[goal_id].feedbacks.append(msg)
            
        
    def mdp_goal_cb(self, msg):
        self.active_goals[msg.goal_id.id] = TaskExecutionStat(mdp_goal = msg,
                                                              feedbacks = [],
                                                              durations = [])

    def mdp_status_cb(self, msg):
        for status in msg.status_list:
            goal_id = status.goal_id.id
            if status.status in self.terminal_states and self.active_goals.has_key(goal_id):
                stat = self.active_goals[goal_id]
                self.finalise_and_add(self.active_goals[goal_id])
                del self.active_goals[goal_id]

    def finalise_and_add(self, stat):
        for i in range(0, len(stat.feedbacks)-1):
            stat.durations.append(stat.feedbacks[i+1].header.stamp - stat.feedbacks[i].header.stamp)
        self.msp.insert(stat)
    
    def main(self):
        # Wait for control-c
        rospy.spin()
        print self.active_goals

if __name__ == '__main__':
    rospy.init_node('mdp_logger')

    logger = Logger()
    logger.main()
    