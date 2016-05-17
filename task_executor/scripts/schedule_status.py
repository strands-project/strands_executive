#!/usr/bin/env python

import rospy
from strands_executive_msgs.msg import *
from random import random
from dateutil.tz import *
from datetime import *
from task_executor.utils import rostime_to_python, rostime_close

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

        



def all_tasks_cb(all_tasks, active_tasks):
    active_ids = set(t.task_id for t in active_tasks.execution_queue)    
    non_active_tasks = [t for t in all_tasks.execution_queue if not t.task_id in active_ids]    
    active_tasks = active_tasks.execution_queue    

    rospy.loginfo("")
    rospy.loginfo("")
    rospy.loginfo(datetime.fromtimestamp(rospy.get_rostime().secs).replace(tzinfo=localtz).strftime("%H:%M:%S %d/%m/%y"))
    if len(active_tasks) > 0:
        rospy.loginfo("Current tasks:")
        for t in active_tasks:
            rospy.loginfo("   %s" % pretty(t))
        rospy.loginfo("Batch started at %s", start_time(active_tasks[0]))        
        rospy.loginfo("       finish by %s", end_time(active_tasks[0]))        
    else:
        rospy.loginfo("No active tasks")

    if len(non_active_tasks) > 0:
        rospy.loginfo("Future tasks (in some order):")
        for t in non_active_tasks:
            rospy.loginfo("   %s after %s" % (pretty(t), rostime_to_python(t.start_after, tz=localtz).strftime("%H:%M:%S %d/%m/%y")))
    else:
        rospy.loginfo("No additional tasks")





if __name__ == '__main__':
    rospy.init_node('schedule_status_printer')

    all_topics = rospy.get_published_topics()
    all_tasks = '/task_executor/all_tasks'
    current_schedule = '/current_schedule'

    use_all_tasks = False
    for topic, type in all_topics:
        if topic == all_tasks:
            use_all_tasks = True
            break

    if use_all_tasks:
        from message_filters import Subscriber, TimeSynchronizer
        ts = TimeSynchronizer([Subscriber(all_tasks, ExecutionStatus), Subscriber(current_schedule, ExecutionStatus)], 1)
        ts.registerCallback(all_tasks_cb)
    else:
        rospy.Subscriber('/current_schedule', ExecutionStatus, callback)
 
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()