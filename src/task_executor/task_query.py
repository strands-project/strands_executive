#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import ros_datacentre_msgs.srv as dc_srv
import ros_datacentre.util as dc_util
from ros_datacentre.message_store import MessageStoreProxy
from strands_executive_msgs.msg import Task, TaskEvent
from datetime import datetime
from task_executor import task_routine

def query_tasks(msg_store, task_id=None, action=None, start_date=None, end_date=None, event=None):
    msg_query = {}
    meta_query = {}

    if task_id is not None:
        msg_query["task.task_id"] = task_id

    if action is not None:
        msg_query["task.action"] = action        

    if event is not None:
        msg_query["event"] = event

    if start_date is not None:
        if end_date is None:
            meta_query["inserted_at"] = {"$gte": start_date}
        else:
            meta_query["inserted_at"] = {"$gte": start_date, "$lte" : end_date}    
    elif end_date is not None:
        meta_query["inserted_at"] = {"$lte": end_date}


    results = msg_store.query(TaskEvent._type, message_query=msg_query, 
                            meta_query=meta_query, single=False)

    return results

def task_event_string(te):
    if te == TaskEvent.ADDED:
        return 'ADDED'
    elif te == TaskEvent.DEMANDED:
        return 'DEMANDED'
    elif te == TaskEvent.TASK_STARTED:
        return 'TASK_STARTED'
    elif te == TaskEvent.NAVIGATION_STARTED:
        return 'NAVIGATION_STARTED'
    elif te == TaskEvent.NAVIGATION_SUCCEEDED:
        return 'NAVIGATION_SUCCEEDED'
    elif te == TaskEvent.NAVIGATION_FAILED:
        return 'NAVIGATION_FAILED'
    elif te == TaskEvent.NAVIGATION_PREEMPTED:
        return 'NAVIGATION_PREEMPTED'
    elif te == TaskEvent.EXECUTION_STARTED:
        return 'EXECUTION_STARTED'
    elif te == TaskEvent.EXECUTION_SUCCEEDED:
        return 'EXECUTION_SUCCEEDED'
    elif te == TaskEvent.EXECUTION_FAILED:
        return 'EXECUTION_FAILED'
    elif te == TaskEvent.EXECUTION_PREEMPTED:
        return 'EXECUTION_PREEMPTED'
    elif te == TaskEvent.CANCELLED_MANUALLY:
        return 'CANCELLED_MANUALLY'
    elif te == TaskEvent.DROPPED:
        return 'DROPPED'    
    elif te == TaskEvent.TASK_FINISHED:
        return 'TASK_FINISHED'    
    elif te == TaskEvent.TASK_FAILED:
        return 'TASK_FAILED'
    elif te == TaskEvent.TASK_SUCCEEDED:
        return 'TASK_SUCCEEDED'
    elif te == TaskEvent.TASK_PREEMPTED:
        return 'TASK_PREEMPTED'    

def summarise(results):

    if len(results) == 0:
        print 'No task events match the query'
        return 
        
    # events in chronological order
    results.sort(key=lambda x: x[1]["inserted_at"])

    output = []
    tid = 0

    for task_event, meta in results:
        if task_event.task.action == '':
            task_event.task.action = 'no action'

        if task_event.event > TaskEvent.DEMANDED:
            if task_event.task.task_id != tid:
                tid = task_event.task.task_id
                output.append('\n')
                       
            output.append(['task %s' % task_event.task.task_id, task_event.task.action, task_event.task.start_node_id, task_event_string(task_event.event), meta["inserted_at"].strftime('%d/%m/%y %H:%M:%S')])

    # http://stackoverflow.com/a/9989441/135585
    col_width = max(len(word) for row in output for word in row) + 4  # padding for row in output:
    for row in output:
        if isinstance(row, list):
            print "".join(word.ljust(col_width) for word in row)
        else:
            print row


def mkdatetime(date_string):
    return datetime.strptime(date_string, '%d/%m/%y %H:%M')

