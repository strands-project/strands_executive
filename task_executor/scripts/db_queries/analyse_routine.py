#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from strands_executive_msgs.msg import Task, TaskEvent
from datetime import datetime, timedelta
from task_executor import task_routine, task_query
from task_executor.utils import rostime_to_python

import argparse
import cmd


class RoutineAnalyser(cmd.Cmd):

    def __init__(self, routine_pairs):
        cmd.Cmd.__init__(self)
        # super(RoutineAnalyser, self).__init__()
        self.routine_pairs = routine_pairs

    def do_merge(self, idx):    
        try:
            idx = int(idx)
            if idx >= 0 and idx < len(self.routine_pairs) - 1:
                print 'merging %s into %s' % (idx, idx+1)
                self.routine_pairs[idx+1] = (self.routine_pairs[idx][0], self.routine_pairs[idx+1][1])
                del self.routine_pairs[idx]
            else:
                print 'invalid routine index, valid range from 0 to %s' % (len(self.routine_pairs) - 2)
        except ValueError, e:
            print 'provided argument was not an int: %s' % idx


    def do_print(self, line):
        for i in range(len(self.routine_pairs)):
            start = rostime_to_python(self.routine_pairs[i][0].time)
            end = rostime_to_python(self.routine_pairs[i][1].time)
            results = task_query.query_tasks(msg_store, 
                        start_date=start,
                        end_date=end,
                        event=[TaskEvent.TASK_STARTED]
                        )

            print 'routine %s: %s to %s, duration: %s, tasks: %s' % (i, start, end, end-start, len(results))
    

    def help_greet(self):
        print '\n'.join([ 'print', 'Print the available routines'])

    def help_merge(self):
        print '\n'.join([ 'merge [idx]', 'Merge the routine at idx into the routine after it'])


    def do_EOF(self, line):
        return True

if __name__ == '__main__':

    rospy.init_node("routine_analysis")

    msg_store = MessageStoreProxy(collection='task_events')

    parser = argparse.ArgumentParser(description='Analyses the task execution behaviour in a routine window. Assumes all task ids are unique in this window.')
    parser.add_argument('start', metavar='S', type=task_query.mkdatetime, nargs='?', 
                   help='start datetime of query, defaults to start of last routine. Formatted "d/m/y H:M" e.g. "06/07/14 06:38"')
    
    parser.add_argument('end', metavar='E', type=task_query.mkdatetime, nargs='?', 
                   help='end datetime of query, defaults to end of last routine or now for an unclosed routine. Formatted "d/m/y H:M" e.g. "06/07/14 06:38"')

    parser.add_argument('-t', '--tasks', type=int, default=1, nargs='?',
                    help='Number of tasks required for a routine to be considered')

    
    args = parser.parse_args()


     
    try:

        results = task_query.query_tasks(msg_store, event=[TaskEvent.ROUTINE_STARTED, TaskEvent.ROUTINE_STOPPED])

        # make sure we start with a ROUTINE_STARTED
        while results[0][0].event == TaskEvent.ROUTINE_STOPPED:
            del results[0]

        routines = []
        start = None
        end = None
       
        # make sure we just have the pairs which are start/end
        for e in results:
            event = e[0]
            if event.event == TaskEvent.ROUTINE_STARTED:
                start = event
            elif event.event == TaskEvent.ROUTINE_STOPPED:
                assert start is not None
                routines.append((start, event))
                start = None


        filtered_routines = []
        for i in range(len(routines)):
            start = rostime_to_python(routines[i][0].time)
            end = rostime_to_python(routines[i][1].time)
            results = task_query.query_tasks(msg_store, 
                        start_date=start,
                        end_date=end,
                        event=[TaskEvent.TASK_STARTED]
                        )

            if len(results) >= args.tasks:
                filtered_routines.append(routines[i])
                # print 'routine %s: %s to %s, duration: %s, tasks: %s' % (len(filtered_routines)-1, start, end, end-start, len(results))

        RoutineAnalyser(filtered_routines).cmdloop()


        # if args.start is None:
        #     # find latest routine start
        #     results = task_query.query_tasks(msg_store, event=TaskEvent.ROUTINE_STARTED)
        #     assert len(results) > 0
        #     start = rostime_to_python(results[-1][0].time)
        # else:
        #     start = args.start

        # end = args.end
        # if end is None:           
        #     results = task_query.query_tasks(msg_store, event=TaskEvent.ROUTINE_STOPPED)
        #     if len(results) > 0:
        #         end_routine = rostime_to_python(results[-1][0].time)               
        #         if start is not None and end_routine > start:
        #             end = end_routine

        
        # results = task_query.query_tasks(msg_store, 
        #                 start_date=start,
        #                 end_date=end,
        #                 event=[TaskEvent.TASK_STARTED, TaskEvent.TASK_FINISHED, TaskEvent.NAVIGATION_SUCCEEDED]
        #                 )

 


        # print 'Tasks Start: %s' % start
        # print 'Tasks End: %s' % end
        # task_query.summarise(results)




    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


        
