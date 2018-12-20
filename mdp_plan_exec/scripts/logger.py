#! /usr/bin/env python

import rospy
from mongodb_store.message_store import MessageStoreProxy
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from strands_navigation_msgs.msg import NavStatistics
from strands_navigation_msgs.srv import PredictEdgeState
from strands_executive_msgs.msg import ExecutePolicyActionFeedback, ExecutePolicyActionGoal
from strands_executive_msgs.msg import TaskExecutionStat, NavExecutionStat

   
class Logger(object):
    def __init__(self):
        self.task_msp = MessageStoreProxy(collection='mdp_task_exec_logs')
        self.nav_msp = MessageStoreProxy(collection='mdp_nav_exec_logs')
        self.current_goal_id = None
        self.active_task_stats = {}
        self.active_nav_stats = {}
        self.terminal_states= [GoalStatus.PREEMPTED, GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.LOST]
        
        mdp_goal_sub = rospy.Subscriber("mdp_plan_exec/execute_policy/goal", ExecutePolicyActionGoal, self.mdp_goal_cb)
        mdp_feedback_sub = rospy.Subscriber("mdp_plan_exec/execute_policy/feedback", ExecutePolicyActionFeedback, self.mdp_feedback_cb)
        mdp_status_sub = rospy.Subscriber("mdp_plan_exec/execute_policy/status", GoalStatusArray, self.mdp_status_cb)
        
        self.nav_stat_sub = rospy.Subscriber("topological_navigation/Statistics", NavStatistics, self.nav_stats_cb)
        self.get_edge_estimates=rospy.ServiceProxy("/topological_prediction/predict_edges", PredictEdgeState)
        rospy.loginfo("Logger started")
        

    def nav_stats_cb(self, msg):
        self.active_nav_stats[self.current_goal_id].nav_stats.append(msg)
        if msg.edge_id in self.current_edge_estimates.edge_ids:
            edge_id = self.current_edge_estimates.edge_ids.index(msg.edge_id)
            self.active_nav_stats[self.current_goal_id].success_probs.append(self.current_edge_estimates.probs[edge_id])
            self.active_nav_stats[self.current_goal_id].expected_times.append(self.current_edge_estimates.durations[edge_id])
        else:
            self.active_nav_stats[self.current_goal_id].success_probs.append(0)
            self.active_nav_stats[self.current_goal_id].expected_times.append(rospy.Duration(0))
        
        

    def mdp_feedback_cb(self, msg):
        goal_id = msg.status.goal_id.id
        if self.active_task_stats.has_key(goal_id):
            self.active_task_stats[goal_id].feedbacks.append(msg)
            
        
    def mdp_goal_cb(self, msg):
        self.current_goal_id = msg.goal_id.id
        self.active_task_stats[self.current_goal_id] = TaskExecutionStat(mdp_goal = msg,
                                                              feedbacks = [],
                                                              durations = [])
        self.active_nav_stats[self.current_goal_id] = NavExecutionStat(mdp_goal = msg,
                                                          nav_stats = [],
                                                          success_probs = [],
                                                          expected_times=[])
        self.current_edge_estimates = self.get_edge_estimates(rospy.Time.now())

        

    def mdp_status_cb(self, msg):
        for status in msg.status_list:
            if status.status in self.terminal_states:
                goal_id = status.goal_id.id
                if self.active_task_stats.has_key(goal_id):
                    rospy.loginfo("Inserting task stat")
                    task_stat = self.active_task_stats[goal_id]
                    self.add_durations(task_stat)
                    self.task_msp.insert(task_stat)
                    del self.active_task_stats[goal_id]
                if self.active_nav_stats.has_key(goal_id):
                    rospy.loginfo("Inserting navigation specific stat")
                    nav_stat = self.active_nav_stats[goal_id]
                    self.nav_msp.insert(nav_stat)
                    del self.active_nav_stats[goal_id]


    def add_durations(self, task_stat):
        for i in range(0, len(task_stat.feedbacks)-1):
            task_stat.durations.append(task_stat.feedbacks[i+1].header.stamp - task_stat.feedbacks[i].header.stamp)
        
    
    def main(self):
        # Wait for control-c
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('mdp_logger')

    logger = Logger()
    logger.main()
    