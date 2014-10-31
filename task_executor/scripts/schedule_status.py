#!/usr/bin/env python

import rospy
from strands_executive_msgs.msg import *
from random import random
from dateutil.tz import *
from datetime import *

def pretty(task):
    if len(task.action) > 0:
        if len(task.start_node_id) > 0:
            return '%s @ %s (task %s)' % (task.action, task.start_node_id, task.task_id)
        else:
            return '%s (task %s)' % (task.action, task.task_id)
    else:
        return 'just navigation to %s (task %s)' % (task.start_node_id, task.task_id)

def start_time(task):
    return datetime.fromtimestamp(task.execution_time.secs)

def callback(data):
    rospy.loginfo("\n\n\n\n\n\n");

    if data.currently_executing:
        rospy.loginfo("Currently executing %s", pretty(data.execution_queue[0]))
        rospy.loginfo("Task started at %s", start_time(data.execution_queue[0]))        
    elif len(data.execution_queue) > 0:
        rospy.loginfo("Waiting to execute %s", pretty(data.execution_queue[0]))
        rospy.loginfo("Execution to start at %s", start_time(data.execution_queue[0]))
    
    if len(data.execution_queue) > 0:
        rospy.loginfo("A further %s tasks queued for execution", len(data.execution_queue) - 1)


if __name__ == '__main__':
    rospy.init_node('schedule_status_printer')
    rospy.Subscriber('/current_schedule', ExecutionStatus, callback)
 
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()