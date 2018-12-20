#!/usr/bin/env python

import rospy
from strands_executive_msgs.msg import *
from random import random
from dateutil.tz import *
from datetime import *
from task_executor.utils import rostime_to_python, rostime_close
import threading
from actionlib_msgs.msg import GoalStatus

lock = threading.Lock()

localtz = tzlocal()


def pretty(task):
    if len(task.action) > 0:
        if len(task.start_node_id) > 0:
            return '%s @ %s (task %s)' % (task.action, task.start_node_id, task.task_id)
        else:
            return '%s (task %s)' % (task.action, task.task_id)
    else:
        return 'just navigation to %s (task %s)' % (task.start_node_id, task.task_id)

def start_time(task):
    return datetime.fromtimestamp(task.execution_time.secs).replace(tzinfo=localtz).strftime("%H:%M:%S %d/%m/%y")

def end_time(task):
    return datetime.fromtimestamp((task.execution_time + task.max_duration).secs).replace(tzinfo=localtz).strftime("%H:%M:%S %d/%m/%y")


def callback(data):
    rospy.loginfo("\n\n\n\n\n\n");

    if data.currently_executing:
        rospy.loginfo("Currently executing %s", pretty(data.execution_queue[0]))
        rospy.loginfo("Task started at %s", start_time(data.execution_queue[0]))        
        rospy.loginfo("      finish by %s", end_time(data.execution_queue[0]))        
    elif len(data.execution_queue) > 0:
        rospy.loginfo("Waiting to execute %s", pretty(data.execution_queue[0]))
        rospy.loginfo("Execution to start at %s", start_time(data.execution_queue[0]))
    
    if len(data.execution_queue) > 0:
        
        rospy.loginfo("A further %s tasks queued for execution", len(data.execution_queue) - 1)

        if rospy.get_param('schedule_verbose', True):
            for i in range(1, min(rospy.get_param('schedule_limit', 15), len(data.execution_queue))):
                rospy.loginfo('%s: %s at %s' % (i, pretty(data.execution_queue[i]), start_time(data.execution_queue[i])))   


goal_string = None
feedback_data = None
all_tasks_data = None
active_tasks_data = None


def policy_goal_callback(policy_goal):
    with lock:
        global goal_string
        goal_string = policy_goal.goal.spec.ltl_task

def policy_feedback_callback(policy_feedback):
    with lock:
        global feedback_data
        feedback_data = policy_feedback
        print_status()

def all_tasks_cb(all_data, active_data):
    with lock:
        global all_tasks_data
        global active_tasks_data
        all_tasks_data = all_data
        active_tasks_data = active_data
        print_status()

def print_status(line_limit=20):
   
    if all_tasks_data is None or active_tasks_data is None:
        return

    active_ids = set(t.task_id for t in active_tasks_data.execution_queue)    
    non_active_tasks = [t for t in all_tasks_data.execution_queue if not t.task_id in active_ids]    
    active_tasks = active_tasks_data.execution_queue    

    line_count = 0

    rospy.loginfo("")
    rospy.loginfo("")
    rospy.loginfo(datetime.fromtimestamp(rospy.get_rostime().secs).replace(tzinfo=localtz).strftime("%H:%M:%S %d/%m/%y"))
    line_count+=1
    if len(active_tasks) > 0:

        rospy.loginfo("Current tasks:")
        line_count+=1
        for t in active_tasks:
            rospy.loginfo("   %s" % pretty(t))
            line_count+=1

        if goal_string is not None:
            rospy.loginfo("Policy:")
            rospy.loginfo('   as LTL string %s' % goal_string)
            line_count+=2
        if feedback_data is not None:
            rospy.loginfo('   executed action: %s, status %s' % (feedback_data.feedback.executed_action, GoalStatus.to_string(feedback_data.feedback.execution_status)))
            rospy.loginfo('   next action: %s' % (feedback_data.feedback.next_action))
            rospy.loginfo('   policy completing with probability %d, expected time %ds' % (feedback_data.feedback.probability, feedback_data.feedback.expected_time.to_sec()))
            line_count+=3



        rospy.loginfo("Batch started at %s", start_time(active_tasks[0]))        
        line_count+=1

        rospy.loginfo("       finish by %s", end_time(active_tasks[0]))        
        line_count+=1

    else:
        rospy.loginfo("No active tasks")
        line_count+=1


    if len(non_active_tasks) > 0:
        rospy.loginfo("Future tasks (in some order):")
        line_count+=1
        printed = 0
        for t in non_active_tasks:
            if line_count > line_limit - 2:
                rospy.loginfo("   ... and another %s tasks not printed" % (len(non_active_tasks) - printed))
                return
            else:
                rospy.loginfo("   %s after %s" % (pretty(t), rostime_to_python(t.start_after, tz=localtz).strftime("%H:%M:%S %d/%m/%y")))
                line_count+=1
                printed+=1
    else:
        rospy.loginfo("No additional tasks")





if __name__ == '__main__':
    rospy.init_node('schedule_status_printer')

    all_topics = rospy.get_published_topics()
    all_tasks = 'task_executor/all_tasks'
    current_schedule = 'current_schedule'

    use_all_tasks = False
    for topic, type in all_topics:
        if topic.endswith(all_tasks):
            use_all_tasks = True
            break



    if use_all_tasks:

        rospy.Subscriber('mdp_plan_exec/execute_policy/goal', ExecutePolicyActionGoal, policy_goal_callback)
        rospy.Subscriber('mdp_plan_exec/execute_policy/feedback', ExecutePolicyActionFeedback, policy_feedback_callback)


        from message_filters import Subscriber, TimeSynchronizer
        ts = TimeSynchronizer([Subscriber(all_tasks, ExecutionStatus), Subscriber(current_schedule, ExecutionStatus)], 1)
        ts.registerCallback(all_tasks_cb)
    else:
        rospy.Subscriber('current_schedule', ExecutionStatus, callback)
 
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()