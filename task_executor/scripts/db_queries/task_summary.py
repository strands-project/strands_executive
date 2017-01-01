#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as ms_util
from mongodb_store.message_store import MessageStoreProxy
from strands_executive_msgs.msg import Task, TaskEvent
from datetime import datetime, timedelta, time, date
from task_executor import task_routine, task_query
from task_executor.utils import rostime_to_python, python_to_rostime, ros_duration_to_string
from task_executor.task_query import task_groups_in_window, daily_windows_in_range, reconstruct_routines, group, task_event_string
from task_executor.routine_analyser import RoutineAnalyser
from copy import copy
from pymongo import Connection
from tabulate import tabulate


import pytz
from dateutil.relativedelta import *
import matplotlib.patches as mpatches
import numpy as np
import matplotlib.pyplot as plt

import argparse
import cmd


def create_task_summary_docs(event_msg_store, event_collection, summary_collection, reprocess = False):
    try:

        event_task_ids = set(event_collection.find().distinct('task.task_id'))
        summary_task_ids = set(summary_collection.find().distinct('task_id'))

        missing_summaries = event_task_ids - summary_task_ids
        print '%s tasks missing summaries' % len(missing_summaries)

        if reprocess:
            missing_summaries = event_task_ids
            print 'reprocessing all %s tasks ' % len(missing_summaries)            

        dead_task_duration = rospy.Duration(60 * 60 * 5)

        now = rospy.get_rostime()

        for task_id in missing_summaries:
            task_events = event_msg_store.query(TaskEvent._type, message_query = {'task.task_id': task_id})            
            task_events.sort(key=lambda x: x[0].time.to_sec())

            first_event = task_events[0][0]
            last_event = task_events[-1][0]
            # check whether this event is done
            last_event_type = last_event.event
      
            if last_event_type == TaskEvent.TASK_SUCCEEDED or \
                last_event_type == TaskEvent.TASK_FAILED or \
                last_event_type == TaskEvent.DROPPED:
                pass
                 
            # else if could still be live
            elif last_event_type != TaskEvent.ROUTINE_STARTED and  last_event_type != TaskEvent.ROUTINE_STOPPED:

                last_event_time = last_event.time
                duration_since_event = now - last_event_time
                if duration_since_event > dead_task_duration:
                    print 'calling time on %s events for task %s with final event type %s in the last %s' % (len(task_events), task_id, task_event_string(last_event_type), ros_duration_to_string(duration_since_event))
                else:
                    print 'ignoring %s events for task %s with final event type %s in the last %s' % (len(task_events), task_id, task_event_string(last_event_type), ros_duration_to_string(duration_since_event))
                    continue
    
            summary_doc = {'task_id': task_id}
            event_sequence = [task_event.event for task_event, meta in task_events]
            events = set(event_sequence)

            summary_doc['started'] = TaskEvent.TASK_STARTED in events
            summary_doc['stopped'] = TaskEvent.TASK_STOPPED in events
            summary_doc['dropped'] = TaskEvent.DROPPED in events
            summary_doc['preempted'] = TaskEvent.TASK_PREEMPTED in events
            summary_doc['succeeded'] = TaskEvent.TASK_SUCCEEDED in events
            summary_doc['failed'] = TaskEvent.TASK_FAILED in events

            if TaskEvent.DEMANDED in events:
                task_type = 'on-demand'
            elif last_event.task.start_after == last_event.task.end_before:
                task_type = 'time-critical'
            else:
                task_type = 'normal'

            summary_doc['task_type'] = task_type
            summary_doc['priority'] = last_event.task.priority
            action = last_event.task.action
            if len(action) == 0:
                action = 'navigation'      
            elif action[0] == '/':
                action = action[1:]

            summary_doc['action'] = action

            summary_doc['ltl_task'] = ' '  in last_event.task.action
            summary_doc['first_event_time'] = rostime_to_python(first_event.time)
            summary_doc['last_event_time'] = rostime_to_python(last_event.time)
            summary_doc['start_count'] = event_sequence.count(TaskEvent.TASK_STARTED)

            # print summary_doc
            summary_collection.update_one({'task.task_id': task_id}, {'$set': summary_doc}, upsert=True)
     


    except rospy.ServiceException, e:
        print "Service call failed: %s"%e



def extend_query(base_query_doc, extension):
    query_doc = copy(base_query_doc)
    for k,v in extension.iteritems():
        query_doc[k] = v
    return query_doc

def get_action_stats(summary_collection, base_query_doc):
    
    total = summary_collection.find(base_query_doc).count()
    some_execution = summary_collection.find(extend_query(base_query_doc, {'started': True})).count()
    succeeded =  summary_collection.find(extend_query(base_query_doc, {'succeeded': True})).count()
    no_execution = summary_collection.find(extend_query(base_query_doc, {'started': False})).count()

    return [total, some_execution, succeeded, no_execution]

def summarise_actions(summary_collection, start_date = None, end_date = None):


    base_query_doc = {}

    if start_date is not None:
        base_query_doc['first_event_time'] = {'$gte': start_date}
    
    if end_date is not None:
        base_query_doc['last_event_time'] = {'$lte': end_date}


    query_doc = copy(base_query_doc)
    query_doc['ltl_task'] = False

    summaries = summary_collection.find(query_doc)

    # print 'Found %s matching summaries' % summaries.count()

    action_stats = []

    for action in summaries.distinct('action'):    
        query_doc['action'] = action
        action_stats.append([action] + get_action_stats(summary_collection, query_doc))

    query_doc = extend_query(base_query_doc, {'ltl_task': True})
    action_stats.append(['ltl'] + get_action_stats(summary_collection, query_doc))
    print tabulate(action_stats, ['action', 'total', 'exe', 'succ', 'drop']) 


def init():
    rospy.init_node("task_summary")
    
    task_event_mongo_host = 'localhost'
    task_event_mongo_port = 62345

    task_summary_mongo_host = 'localhost'
    task_summary_mongo_port = 62345

    db_name = 'message_store'
    event_col_name = 'task_events_unique'        
    event_msg_store = MessageStoreProxy(database=db_name, collection=event_col_name)
    
    task_event_mongo_client = Connection(task_event_mongo_host, task_event_mongo_port)
    event_collection = task_event_mongo_client[db_name][event_col_name]
    
    summary_col_name = 'task_summaries'
    task_summary_mongo_client = Connection(task_summary_mongo_host, task_summary_mongo_port)
    summary_collection = task_summary_mongo_client[db_name][summary_col_name]


    tz = pytz.timezone(pytz.country_timezones['gb'][0])

    analysis_start = datetime(2016,5,23,5,00,tzinfo=tz)
    # analysis_end = datetime(2016,6,6,23,00,tzinfo=tz)
    analysis_end = datetime(2016,8,10,23,00,tzinfo=tz)

    create_task_summary_docs(event_msg_store, event_collection, summary_collection, reprocess=False)
    summarise_actions(summary_collection, start_date=analysis_start, end_date=analysis_end)    



if __name__ == '__main__':
    init()
    


        
