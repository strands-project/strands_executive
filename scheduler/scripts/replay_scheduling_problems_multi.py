#!/usr/bin/env python

import rospy
import argparse
import numpy

from mongodb_store.message_store import MessageStoreProxy
from mongodb_store_msgs.msg import StringPairList
from datetime import datetime
from strands_executive_msgs.msg import Task
from strands_executive_msgs.srv import GetSchedule
from threading import Thread


def mkdatetime(date_string):
    return datetime.strptime(date_string, '%d/%m/%y %H:%M')

if __name__ == "__main__":
    rospy.init_node("schedule_replay")

    parser = argparse.ArgumentParser(description='Replays saved scheduling problems from mongodb_store.')

    parser.add_argument('instances', metavar='I', type=int, nargs='?', default=0,
                   help='Number of instances of scheduler to use.')
    
    parser.add_argument('start', metavar='S', type=mkdatetime, nargs='?', default=datetime.utcfromtimestamp(0),
                   help='start datetime of query, defaults to no start. Formatted "d/m/y H:M" e.g. "06/07/14 06:38"')
    
    parser.add_argument('end', metavar='E', type=mkdatetime, nargs='?', default=datetime.utcnow(),
                   help='end datetime of query, defaults to now. Formatted "d/m/y H:M" e.g. "06/07/14 06:38"')

    
    args = parser.parse_args()


    
    msg_store = MessageStoreProxy(collection='scheduling_problems')
    meta_query = {}
    meta_query["inserted_at"] = {"$gte": args.start, "$lte" : args.end} 

    scheduling_problems = msg_store.query(StringPairList._type, meta_query=meta_query)
    scheduling_problems.sort(key=lambda x: x[1]["inserted_at"])

    print "%s problems" % len(scheduling_problems)
    print "First problem on %s " % scheduling_problems[0][1]["inserted_at"].strftime('%d/%m/%y %H:%M:%S')
    print " Last problem on %s " % scheduling_problems[-1][1]["inserted_at"].strftime('%d/%m/%y %H:%M:%S')


    services = []

    max_int = 1 + args.instances

    for i in range(1, max_int):
        # service for scheduler
        schedule_srv_name = 'get_schedule_' + str(i)
        rospy.logdebug('Waiting for %s service' % schedule_srv_name)
        rospy.wait_for_service(schedule_srv_name)
        services.append(rospy.ServiceProxy(schedule_srv_name, GetSchedule))

    problems = []

    count = 0
    for message, meta in scheduling_problems:
        tasks = []
        earliest_start = rospy.get_rostime()


        for pair in message.pairs:
            #task = msg_store.query_id(message.pairs[0].second, Task._type)[0]
            task = msg_store.query_id(pair.second, Task._type)[0]
            tasks.append(task)

            if task.start_after < earliest_start:
                earliest_start = task.start_after

        #set earliest task to start at 0
        for task in tasks:
            task.start_after = task.start_after - earliest_start
            task.end_before = task.end_before - earliest_start

        # reset to 0
        earliest_start = rospy.Time()

        threads = []
        for service in services:
            def call():
                service(tasks, earliest_start, 0)
            threads.append(Thread(target=call))
            threads[-1].start()
            rospy.sleep(0.1)

        for thread in threads:
            thread.join()
            rospy.sleep(0.1)

        count += 1
        print '%s/%s' % (count, len(scheduling_problems))

