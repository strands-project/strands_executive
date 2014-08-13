#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from strands_executive_msgs.msg import Task, TaskEvent
from datetime import datetime, timedelta
from task_executor import task_routine, task_query
import argparse


if __name__ == '__main__':

    rospy.init_node("execution_time")

    msg_store = MessageStoreProxy(collection='task_events')

    parser = argparse.ArgumentParser(description='Prints a summary of tasks executed within the queried time window.')
    parser.add_argument('start', metavar='S', type=task_query.mkdatetime, nargs='?', default=datetime.utcfromtimestamp(0),
                   help='start datetime of query, defaults to no start. Formatted "d/m/y H:M" e.g. "06/07/14 06:38"')
    
    parser.add_argument('end', metavar='E', type=task_query.mkdatetime, nargs='?', default=datetime.utcnow(),
                   help='end datetime of query, defaults to now. Formatted "d/m/y H:M" e.g. "06/07/14 06:38"')

    
    args = parser.parse_args()


     
    try:

        start = args.start
        end = args.end

        results = task_query.query_tasks(msg_store, 
                        start_date=start,
                        end_date=end,
                        event=[TaskEvent.TASK_STARTED, TaskEvent.TASK_FINISHED, TaskEvent.NAVIGATION_SUCCEEDED]
                        )


        duration = timedelta()
        charge_wait_duration = timedelta()

        started_task_event = TaskEvent()
        count = 0
        dubious = []
        charge_wait_count = 0
        unstarted_count = 0
        start_count = 0
        day_durations = {}



        for i in range(0, len(results)):

            if i > 0:
                previous = results[i-1][0]
            
            task_event = results[i][0]
            
            if i < len(results) - 1:
                next = results[i+1][0]

            start = False
            end = False

            if task_event.event == TaskEvent.TASK_STARTED or task_event.task.task_id != previous.task.task_id:
                start = True
            elif (task_event.event == TaskEvent.TASK_FINISHED or task_event.task.task_id != next.task.task_id or (task_event.task.action == '' and task_event.event == TaskEvent.NAVIGATION_SUCCEEDED)) and started_task_event.task.task_id != 0:
                end = True

            if start:
                started_task_event = task_event
                start_count += 1
            elif end:

                # if we're closing the previous event
                if task_event.task.task_id == started_task_event.task.task_id:
            
                    end_time = datetime.utcfromtimestamp(task_event.time.to_sec())
                    task_duration = end_time - datetime.utcfromtimestamp(started_task_event.time.to_sec())

                    if task_event.task.action == 'wait_action' and task_event.task.start_node_id == 'ChargingPoint':
                        charge_wait_duration += task_duration
                        charge_wait_count += 1
                    else:
                        
                        if task_duration > timedelta(hours=3):
                            dubious.append((started_task_event, task_event))
                            # task_query.print_event(started_task_event)
                            # task_query.print_event(task_event)                            
                            # print '%s\n' % task_duration                  
                        else:

                            task_date = end_time.date()

                            if task_date in day_durations:
                                day_durations[task_date].append(task_duration)
                            else:
                                day_durations[task_date] = [task_duration]

                            duration += task_duration
                            count += 1

                    # make sure we don't look for another end to this task
                    started_task_event = TaskEvent()

                else:
                    # print 'Finished an unstarted task: %s' % task_event
                    unstarted_count += 1
                    # task_query.print_event(started_task_event)
                    # task_query.print_event(task_event)
                    # print '\n'
            
        start = results[0][0].time
        end = results[-1][0].time


        print 'Starts: %s' % start_count
        print 'Ends: %s (%s + %s + %s)' % (count + charge_wait_count  +  len(dubious), count, charge_wait_count, len(dubious))

        print 'Unstarted: %s' % unstarted_count
        print 'Dubious: %s' % len(dubious)

        print 'Tasks Completed: %s' % count        
        print 'Task Duration: %s' % duration

        print 'Charge Waits Finished: %s' % charge_wait_count        
        print 'Charge Waits Duration: %s' % charge_wait_duration

        print 'Tasks Start: %s' % datetime.utcfromtimestamp(start.to_sec())
        print 'Tasks End: %s' % datetime.utcfromtimestamp(end.to_sec())

        print 'Total activity span: %s ' % timedelta(seconds=(end - start).to_sec())

        days = day_durations.keys()
        days.sort()

        print 'Tasks and duration each day'
        total_possible = timedelta()
        for day in days:
            total_possible += timedelta(hours=9)
            durations = day_durations[day]
            total = timedelta()
            for d in durations:
                total += d
            print '%s:\t%s\t%s' % (day, len(durations), total)

        print 'Autonomy percentage: %.2f' % ((duration.total_seconds()/total_possible.total_seconds()) * 100)



    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


        
