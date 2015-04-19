#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from strands_executive_msgs.msg import Task, TaskEvent
from datetime import datetime
from task_executor import task_routine, task_query
import argparse


if __name__ == '__main__':

    rospy.init_node("summarise_tasks")


    msg_store = MessageStoreProxy(collection='task_events')

    parser = argparse.ArgumentParser(description='Prints a summary of tasks executed within the queried time window.')
    parser.add_argument('start', metavar='S', type=task_query.mkdatetime, nargs='?', default=datetime.utcfromtimestamp(0),
                   help='start datetime of query, defaults to the midnight just passed. Formatted "d/m/y H:M" e.g. "06/07/14 06:38"')
    parser.add_argument('end', metavar='E', type=task_query.mkdatetime, nargs='?',
                   help='end datetime of query, defaults to no end. Formatted "d/m/y H:M" e.g. "06/07/14 06:38"')

    
    args = parser.parse_args()


     
    try:


        start = args.start
        end = args.end

        results = task_query.query_tasks(msg_store, 
                        # event=TaskEvent.ADDED, 
                        # action='check_door',
                        start_date=start,
                        end_date=end,
                        )
        # print len(results)
        # print results
        task_query.summarise(results)

        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


        
