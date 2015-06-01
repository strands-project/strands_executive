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
    results.sort(key=lambda x: x[0].time.to_sec())

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
    elif te == TaskEvent.ROUTINE_STARTED:
        return 'ROUTINE_STARTED'    
    elif te == TaskEvent.ROUTINE_STOPPED:
        return 'ROUTINE_STOPPED'    


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
    if event.event == TaskEvent.TASK_STARTED:
        return True
    elif len(open_group) == 0:
        return False
    # else:
    #     return event.task.task_id != open_group[-1].task.task_id

def group(results):
    """ 
    Groups events into individual executions
    """

    if len(results) == 0:
        return []

    groups = []
    open_group = [results[0][0]]

    for event, meta in results[1:]:
        # start of an event group
        if start_of_new_group(event, open_group):
            groups.append(open_group)
            open_group = []
        
        open_group.append(event)

    # any remaining bits
    if len(open_group) > 0:
        groups.append(open_group)

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

def autonomy_time(window_start, window_end, events):
    if len(events) == 0:
        print 'No task events match the query'
        return 

    event_groups = group(events)
    execution_duration = timedelta()
    for event_group in event_groups:
        # we shouldn't count waiting around as autonomy
        end_event = event_group[-1]
        if event_group[0].task.action == 'wait_action':
            for event in event_group:
                if event.event < TaskEvent.EXECUTION_STARTED:
                    end_event = event
                else:
                    break

        execution_duration += rostime_to_python(end_event.time) - rostime_to_python(event_group[0].time)

    return execution_duration


# def aggregate(results):

#     duration = timedelta()
#     charge_wait_duration = timedelta()

#     started_task_event = TaskEvent()
#     count = 0
#     dubious = []
#     charge_wait_count = 0
#     unstarted_count = 0
#     start_count = 0
#     day_durations = {}

#     for i in range(0, len(results)):

#         if i > 0:
#             previous = results[i-1][0]
#         else:
#             previous = None

#         task_event = results[i][0]
        
#         if i < len(results) - 1:
#             next = results[i+1][0]
#         else:
#             next = None

#         start = False
#         end = False

#         if task_event.event == TaskEvent.TASK_STARTED or task_event.task.task_id != previous.task.task_id:
#             start = True
#         elif (task_event.event == TaskEvent.TASK_FINISHED or task_event.task.task_id != next.task.task_id or (task_event.task.action == '' and task_event.event == TaskEvent.NAVIGATION_SUCCEEDED)) and started_task_event.task.task_id != 0:
#             end = True

#         if start:
#             started_task_event = task_event
#             start_count += 1
#         elif end:

#             # if we're closing the previous event
#             if task_event.task.task_id == started_task_event.task.task_id:
        
#                 end_time = datetime.utcfromtimestamp(task_event.time.to_sec())
#                 task_duration = end_time - datetime.utcfromtimestamp(started_task_event.time.to_sec())

#                 if task_event.task.action == 'wait_action' and task_event.task.start_node_id == 'ChargingPoint':
#                     charge_wait_duration += task_duration
#                     charge_wait_count += 1
#                 else:
                    
#                     if task_duration > timedelta(hours=3):
#                         dubious.append((started_task_event, task_event))
#                         # task_query.print_event(started_task_event)
#                         # task_query.print_event(task_event)                            
#                         # print '%s\n' % task_duration                  
#                     else:

#                         task_date = end_time.date()

#                         if task_date in day_durations:
#                             day_durations[task_date].append(task_duration)
#                         else:
#                             day_durations[task_date] = [task_duration]

#                         duration += task_duration
#                         count += 1

#                 # make sure we don't look for another end to this task
#                 started_task_event = TaskEvent()

#             else:
#                 # print 'Finished an unstarted task: %s' % task_event
#                 unstarted_count += 1
#                 # task_query.print_event(started_task_event)
#                 # task_query.print_event(task_event)
#                 # print '\n'
        
#     start = results[0][0].time
#     end = results[-1][0].time


#     print 'Starts: %s' % start_count
#     print 'Ends: %s (%s + %s + %s)' % (count + charge_wait_count  +  len(dubious), count, charge_wait_count, len(dubious))

#     print 'Unstarted: %s' % unstarted_count
#     print 'Dubious: %s' % len(dubious)

#     print 'Tasks Completed: %s' % count        
#     print 'Task Duration: %s' % duration

#     print 'Charge Waits Finished: %s' % charge_wait_count        
#     print 'Charge Waits Duration: %s' % charge_wait_duration

#     print 'Tasks Start: %s' % datetime.utcfromtimestamp(start.to_sec())
#     print 'Tasks End: %s' % datetime.utcfromtimestamp(end.to_sec())

#     print 'Total activity span: %s ' % timedelta(seconds=(end - start).to_sec())

#     days = day_durations.keys()
#     days.sort()

#     print 'Tasks and duration each day'
#     total_possible = timedelta()
#     for day in days:
#         total_possible += timedelta(hours=9)
#         durations = day_durations[day]
#         total = timedelta()
#         for d in durations:
#             total += d
#         print '%s:\t%s\t%s' % (day, len(durations), total)

#     print 'Autonomy percentage: %.2f' % ((duration.total_seconds()/total_possible.total_seconds()) * 100)

