#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from strands_executive_msgs.msg import Task, TaskEvent
from datetime import datetime, timedelta
from task_executor import task_routine
from task_executor.utils import rostime_to_python







def remove_duplicates(results):
    seen = set()
    seen_add = seen.add
    return [ x for x in results if x[0] not in seen and not seen_add(x[0])]

def query_tasks(msg_store, task_id=None, action=None, start_date=None, end_date=None, event=None):
    msg_query = {}
    meta_query = {}


    if task_id is not None:
        msg_query["task.task_id"] = task_id

    if action is not None:
        msg_query["task.action"] = action        

    if event is not None:
        if not isinstance(event, list):
            event = [event]

        # create a disjunctive query over events
        event_qs = []
        for e in event:
            event_qs.append({'event': e})        

        msg_query['$or'] = event_qs

    if start_date is not None:
        if end_date is None:
            meta_query["inserted_at"] = {"$gte": start_date}
        else:
            meta_query["inserted_at"] = {"$gte": start_date, "$lte" : end_date}    
    elif end_date is not None:
        meta_query["inserted_at"] = {"$lte": end_date}


    results = msg_store.query(TaskEvent._type, message_query=msg_query, 
                            meta_query=meta_query, single=False)


    # results.sort(key=lambda x: x[1]["inserted_at"])
    # results.sort(key=lambda x: x[0].time.to_sec())

    return results

def sort_results_by_logged_time(results):
    results.sort(key=lambda x: x[0].time.to_sec())


    


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
    elif te == TaskEvent.ROUTINE_STARTED:
        return 'ROUTINE_STARTED'    
    elif te == TaskEvent.ROUTINE_STOPPED:
        return 'ROUTINE_STOPPED'    
    elif te == TaskEvent.TASK_STOPPED:
        return 'TASK_STOPPED'    


def format_event(task_event):
    """ Prints a single event """
    return 'task %s\t\t%s\t%s\t%s\t\t%s' % (task_event.task.task_id, task_event.task.action, task_event.task.start_node_id, task_event_string(task_event.event), datetime.fromtimestamp(task_event.time.to_sec()).strftime('%d/%m/%y %H:%M:%S'))


def print_event(task_event):
    """ Prints a single event """
    print format_event(task_event)



def event_time(task_event):
    return datetime.utcfromtimestamp(task_event.time.to_sec()).strftime('%d/%m/%y %H:%M:%S')

def mktime(time_string):
    return datetime.strptime(time_string, '%H:%M').time()


def mkdatetime(date_string):
    return datetime.strptime(date_string, '%d/%m/%y %H:%M')


def start_of_new_group(event, open_group):

    # events should come in order by logged id then by logged time. 
    # this means they should roughly be in order so we can 
    # distinguish them based on event id, as this as this should 
    # always increase within a group. the exception is 1->2 which 
    # is added to demanded which could happen sequentially but should
    # be treated as separate groups

    if len(open_group) == 0:
        return False
    elif event.event <= open_group[-1].event:
        return True
    elif event.event == TaskEvent.DEMANDED and open_group[-1].event == TaskEvent.ADDED:
        return True    
    elif event.task.task_id != open_group[-1].task.task_id:
        return True    
    else:
        return False

def print_group(group):
    print '['
    for event in group:
        print event.task.task_id, event.event, rostime_to_python(event.task.start_after), rostime_to_python(event.time)
    print ']'

def group(results, group_length = 0):
    """ 
    Groups results of query_tasks into individual execution groups. 
    If group_length is not 0, only groups of this length are returned
    """

    if len(results) == 0:
        return []

    results.sort(key=lambda x: x[0].time.to_sec())
    results.sort(key=lambda x: x[0].task.task_id)

    groups = []
    open_group = [results[0][0]]

    for event, meta in results[1:]:
        # print event.task.task_id, event.event, rostime_to_python(event.task.start_after), rostime_to_python(event.time)

        # start of an event group
        if start_of_new_group(event, open_group):            
            if group_length == 0 or len(open_group) == group_length:
                groups.append(open_group)
            open_group = []
        
        open_group.append(event)

    # any remaining bits
    if len(open_group) > 0:
        if group_length == 0 or len(open_group) == group_length:
            groups.append(open_group)

    # now sort so the groups are in execution order
    # this is done using the logged time of the last event in the gourp
    groups.sort(key=lambda g: g[-1].time.to_sec())

    # for g in groups:
    #     print_group(g)

    return groups


def aggregate(results):
    if len(results) == 0:
        print 'No task events match the query'
        return 

    event_groups = group(results)

    by_action = {}

    for event_group in event_groups:
        action = event_group[0].task.action
        if not action in by_action:
            # [success, failed]
            by_action[action] = [0, 0]

        if any(te for te in event_group if te.event == TaskEvent.TASK_SUCCEEDED):
            by_action[action][0] += 1
        else:
            by_action[action][1] += 1

    for action, stats in by_action.iteritems():
        print '%s: %s/%s' % (action, stats[0], sum(stats))





def executions(results):

    if len(results) == 0:
        print 'No task events match the query'
        return 

    groups = group(results)
        
    output = []
 
    for exec_group in groups:
        output.append('\n')
        for task_event in exec_group:
            if task_event.task.action == '':
                task_event.task.action = 'no action'            
                                   
            output.append(['task %s' % task_event.task.task_id, task_event.task.action, task_event.task.start_node_id, task_event_string(task_event.event), datetime.utcfromtimestamp(task_event.time.to_sec()).strftime('%d/%m/%y %H:%M:%S')])

    # http://stackoverflow.com/a/9989441/135585
    col_width = max(len(word) for row in output for word in row) + 4  # padding for row in output:
    for row in output:
        if isinstance(row, list):
            print "".join(word.ljust(col_width) for word in row)
        else:
            print row

def autonomy_time(window_start, window_end, msg_store):

    execution_duration = timedelta()

    for event_group in task_groups_in_window(window_start, window_end, msg_store):
    
        i = 0
        while i < len(event_group) - 1 and event_group[i].event < TaskEvent.TASK_STARTED:
            i+=1

        # we shouldn't count waiting around as autonomy
        start_event = event_group[i]
        end_event = event_group[-1]
        if start_event.task.action == 'wait_action':
            for event in event_group:
                if event.event < TaskEvent.EXECUTION_STARTED:
                    end_event = event
                else:
                    # print 'ignoring wait time'
                    break

        if end_event.time < start_event.time:
            print_group(event_group)

        execution_duration += rostime_to_python(end_event.time) - rostime_to_python(start_event.time)

    return execution_duration



def task_groups_in_window(window_start, window_end, msg_store, event=None):
    """A generator which returns groups of events for tasks during the given window.
    """
    results = query_tasks(msg_store, 
        event=event,             
        start_date=window_start,
        end_date=window_end)

    group_length = 0
    if event is not None:
        group_length = len(event)

    groups = group(results, group_length)
    for g in groups:
        yield g


def daily_windows_in_range(daily_start_time, daily_end_time, window_start, window_end, days_off = []):
    """A generator which returns datetime pairs for the start and end 
    points of a daily internal during the overall time window.
    """
    # coerce window start into acceptable routine range
    #  daily_start, and daily_end are the returned pairs

    # if greater than the end of the day, start at daily_start on the next day
    if window_start.time() > daily_end_time:
        daily_start = datetime.combine((window_start + timedelta(days=1)).date(), daily_start_time)
    # if less that then start of the day, start at daily start on this day
    elif window_start.time() < daily_start_time:
        daily_start = datetime.combine(window_start.date(), daily_start_time)
    else:
        daily_start = window_start

    daily_end = datetime.combine(daily_start.date(), daily_end_time)


    while daily_end < window_end:

        day = daily_start.strftime("%A")
        date = daily_start.date()

        if day not in days_off and date not in days_off:
            yield daily_start, daily_end

        daily_start = datetime.combine((daily_start + timedelta(days=1)).date(), daily_start_time)
        daily_end = datetime.combine((daily_end + timedelta(days=1)).date(), daily_end_time)

    # catch the final loop 
    if daily_start < window_end and window_end < daily_end:
        daily_end = window_end

        day = daily_start.strftime("%A")
        date = daily_start.date()

        if day not in days_off and date not in days_off:
            yield daily_start, daily_end



def reconstruct_routines(results, min_tasks=1, allow_open=True):
    """Pair up routine starts and ends. This typically requires all task events to infer end of earlier unclosed routines.
    """

    routines = []
    start = None
    task_count = 0

    # make sure we just have the pairs which are start/end
    for i in range(len(results)):
        event = results[i][0]
        if event.event == TaskEvent.ROUTINE_STARTED:
            if start is not None:
                # need to close previous routine
                dummy_end = TaskEvent(event = TaskEvent.ROUTINE_STOPPED, task = Task(), time = results[i-1][0].time)
                if task_count >= min_tasks:
                    print 'closing unterminated routine'
                    routines.append((start, dummy_end))
            start = event
            task_count = 0
        elif event.event == TaskEvent.ROUTINE_STOPPED and start is not None:
            if task_count >= min_tasks:
                routines.append((start, event))
            start = None
        else:
            task_count += 1

        # if this is the last event and it's not an event, close routine here
        if i == len(results) - 1 and start is not None:
            dummy_end = TaskEvent(event = TaskEvent.ROUTINE_STOPPED, task = Task(), time = event.time)
            if task_count >= min_tasks:
                print 'closing unterminated routine at the end'
                routines.append((start, dummy_end))


    return routines

