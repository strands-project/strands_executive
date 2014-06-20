#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import ros_datacentre_msgs.srv as dc_srv
import ros_datacentre.util as dc_util
from ros_datacentre.message_store import MessageStoreProxy
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


        duration = rospy.Duration()
        charge_wait_duration = rospy.Duration()

        started_task_event = TaskEvent()
        count = 0
        dubious = []
        charge_wait_count = 0
        unstarted_count = 0
        start_count = 0

        for task_event, meta in results:

            if task_event.event == TaskEvent.TASK_STARTED:
                start_time = task_event.time                
                started_task_event = task_event
                start_count += 1

            elif task_event.event == TaskEvent.TASK_FINISHED or \
                    (task_event.task.action == '' and task_event.event == TaskEvent.NAVIGATION_SUCCEEDED):

                # if we're closing the previous event
                if task_event.task.task_id == started_task_event.task.task_id:
                    task_duration = task_event.time - start_time

                    if task_event.task.action == 'wait_action' and task_event.task.start_node_id == 'ChargingPoint':
                        charge_wait_duration += task_duration
                        charge_wait_count += 1
                    else:
                        if duration.to_sec() > 60 * 60 * 10:
                            dubious.append((started_task_event, task_event))
                        else:
                            duration += task_duration
                            count += 1
                    
                else:
                    # print 'Finished an unstarted task: %s' % task_event
                    unstarted_count += 1
            
        start = results[0][0].time
        end = results[-1][0].time


        print 'Starts: %s' % start_count
        print 'Ends: %s (%s + %s + %s)' % (count + charge_wait_count  +  len(dubious), count, charge_wait_count, len(dubious))

        print 'Unstarted: %s' % unstarted_count
        print 'Dubious: %s' % len(dubious)

        print 'Finished: %s' % count        
        print 'Duration: %s' % timedelta(seconds=duration.to_sec())

        print 'Charge Wait Finished: %s' % charge_wait_count        
        print 'Charge Wait Duration: %s' % timedelta(seconds=charge_wait_duration.to_sec())

        print 'Start: %s' % datetime.utcfromtimestamp(start.to_sec())
        print 'End: %s' % datetime.utcfromtimestamp(end.to_sec())

        print 'Total activity: %s ' % timedelta(seconds=(end - start).to_sec())


    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


        
