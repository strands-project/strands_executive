#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from strands_executive_msgs.msg import Task, TaskEvent
from datetime import datetime
from task_executor import task_routine, task_query
from task_executor.utils import rostime_to_python
import argparse


if __name__ == '__main__':

    rospy.init_node("routine_times")


    msg_store = MessageStoreProxy(collection='task_events')

    parser = argparse.ArgumentParser(description='Prints a a list of when the task routine started and stopped.')
    parser.add_argument('start', metavar='S', type=task_query.mkdatetime, nargs='?', default=datetime.utcfromtimestamp(0),
                   help='start datetime of query, defaults to the midnight just passed. Formatted "d/m/y H:M" e.g. "06/07/14 06:38"')
    parser.add_argument('end', metavar='E', type=task_query.mkdatetime, nargs='?',
                   help='end datetime of query, defaults to no end. Formatted "d/m/y H:M" e.g. "06/07/14 06:38"')

    
    args = parser.parse_args()


     
    try:


        start = args.start
        end = args.end

        results = task_query.query_tasks(msg_store, 
                        event=[TaskEvent.ROUTINE_STARTED, TaskEvent.ROUTINE_STOPPED],
                        start_date=start,
                        end_date=end,
                        )
        
        for event in results:
            task_event = event[0]
            event_type = task_event.event
            if event_type == TaskEvent.ROUTINE_STARTED:
                last_start = rostime_to_python(task_event.time)
                append = ''
            else:
                end_time = rostime_to_python(task_event.time)
                append = ', so the robot ran for %s\n' % (end_time - last_start)

            print('%s %s%s'%(task_query.task_event_string(event_type), task_query.event_time(task_event), append))
  


        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


        
