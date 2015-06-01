#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from strands_executive_msgs.msg import Task, TaskEvent
from datetime import datetime, timedelta, time, date
from task_executor import task_routine, task_query
from task_executor.utils import rostime_to_python

import argparse
import cmd


class RoutineAnalyser(cmd.Cmd):

    def __init__(self, msg_store, routine_pairs, daily_start = None, daily_end = None):
        cmd.Cmd.__init__(self)
        # super(RoutineAnalyser, self).__init__()
        self.routine_pairs = routine_pairs
        self.msg_store = msg_store
        self.daily_start = daily_start if daily_start is not None else time(0,0)
        self.daily_end = daily_end if daily_end is not None else time(23,59)
        # hand code for now
        self.days_off = ['Saturday', 'Sunday', date(2015, 5, 25), date(2015, 5, 4) ]


    def check_idx(self, idx):
        if idx < 0 or idx >= len(self.routine_pairs):
            print 'idx must be greater than 0 and less than %s' % len(self.routine_pairs)
            return False
        else:
            return True


    def do_merge(self, idx):    
        try:
            if idx == 'all':
                self.routine_pairs = [(self.routine_pairs[0][0], self.routine_pairs[-1][1])]
            else:
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


    def do_summarise(self, idx):    

        try:
            idx = int(idx)

            if not self.check_idx(idx):
                return


            window_start = rostime_to_python(self.routine_pairs[idx][0].time)
            window_end = rostime_to_python(self.routine_pairs[idx][1].time)

            results = task_query.query_tasks(self.msg_store, 
                        event=range(TaskEvent.TASK_STARTED, TaskEvent.ROUTINE_STARTED), 
                        start_date=window_start,
                        end_date=window_end,
                        )
            task_query.aggregate(results)


        except ValueError, e:
            print 'provided argument was not an int: %s' % idx


    def all_task_events_in_window(self, window_start, window_end, action = None):
        return task_query.query_tasks(self.msg_store, 
            event=range(TaskEvent.TASK_STARTED, TaskEvent.ROUTINE_STARTED), 
            action=action,
            start_date=window_start,
            end_date=window_end,
            )



    def autonomy_durations(self, daily_start, daily_end):
        daily_events = self.all_task_events_in_window(daily_start, daily_end)
        autonomy_duration = task_query.autonomy_time(daily_start, daily_end, daily_events)
        day_duration = daily_end - daily_start 
        return autonomy_duration, day_duration


    def autonomy_day(self, daily_start, daily_end):    
        day = daily_start.strftime("%A")
        date = daily_start.date()

        if day not in self.days_off and date not in self.days_off:

            autonomy_duration, day_duration = self.autonomy_durations(daily_start, daily_end)
            autonomy_percentage = (autonomy_duration.total_seconds() / day_duration.total_seconds()) * 100.0
            return autonomy_duration, day_duration            

            print '%s: %s from %s -> %s' % (daily_start.date(), autonomy_duration, day_duration, autonomy_percentage)
        else:
            print 'Skipping %s %s' % (day, date)
            return timedelta(), timedelta()

    def do_autonomy(self, idx):    
        try:
            idx = int(idx)

            if not self.check_idx(idx):
                return

            window_start = rostime_to_python(self.routine_pairs[idx][0].time)
            window_end = rostime_to_python(self.routine_pairs[idx][1].time)

            print 'start', window_start
            print 'end', window_end

            # coerce window start into acceptable routine range

            # if greater than the end of the day, start at daily_start on the next day
            if window_start.time() > self.daily_end:
                daily_start = datetime.combine((window_start + timedelta(days=1)).date(), self.daily_start)
            # if less that then start of the day, start at daily start on this day
            elif window_start.time() < self.daily_start:
                daily_start = datetime.combine(window_start.date(), self.daily_start)
            else:
                daily_start = window_start

            daily_end = datetime.combine(daily_start.date(), self.daily_end)

            total_autonomy_duration = timedelta(seconds=0)
            total_life_duration = timedelta(seconds=1)

            while daily_end < window_end:

                autonomy_duration, day_duration = self.autonomy_day(daily_start, daily_end)
                total_autonomy_duration += autonomy_duration
                total_life_duration += day_duration

                daily_start = datetime.combine((daily_start + timedelta(days=1)).date(), self.daily_start)
                daily_end = datetime.combine((daily_end + timedelta(days=1)).date(), self.daily_end)


            # catch the final loop 
            if daily_start < window_end and window_end < daily_end:
                daily_end = window_end

                autonomy_duration, day_duration = self.autonomy_day(daily_start, daily_end)
                total_autonomy_duration += autonomy_duration
                total_life_duration += day_duration


            total_autonomy_percentage = (total_autonomy_duration.total_seconds() / total_life_duration.total_seconds()) * 100.0
            print 'A: %s' % total_autonomy_percentage

        except ValueError, e:
            print 'provided argument was not an int: %s' % idx


    def do_executions(self, line):    
        try:

            tokens = line.split(' ')
            idx = tokens[0]

            if len(tokens) > 1 and len(tokens[1]) > 0:
                action = tokens[1]
                print 'only showing action: %s' % action
            else:
                action = None


            idx = int(idx)

            if not self.check_idx(idx):
                return

            print 'executions in routine %s' % idx
            window_start = rostime_to_python(self.routine_pairs[idx][0].time)
            window_end = rostime_to_python(self.routine_pairs[idx][1].time)

            results = task_query.query_tasks(self.msg_store, 
                        event=range(TaskEvent.TASK_STARTED, TaskEvent.ROUTINE_STARTED), 
                        action=action,
                        start_date=window_start,
                        end_date=window_end,
                        )
            task_query.executions(results)


        except ValueError, e:
            print 'provided argument was not an int: %s' % idx
            print e    

    def help_print(self):
        print '\n'.join([ 'print', 'Print the available routines'])

    def help_merge(self):
        print '\n'.join([ 'merge [idx | all]', 'Merge routine idx (int) into the routine after it. If idx is the string all, then merge all routines together.'])

    def help_executions(self):
        print '\n'.join([ 'executions [idx]', 'Show all task executions in routine idx (int).'])

    def help_summarise(self):
        print '\n'.join([ 'summarise [idx]', 'Summarise task executions in routine idx (int).'])

    def help_autonomy(self):
        print '\n'.join([ 'autonomy [idx]', 'Print autonomy percentage for routine idx (int).'])

    def do_EOF(self, line):
        return True

    def help_EOF(self):
        print 'Exit (CTRL-D)'


if __name__ == '__main__':

    rospy.init_node("routine_analysis")

    msg_store = MessageStoreProxy(collection='task_events')

    parser = argparse.ArgumentParser(description='Analyses the task execution behaviour in a routine window. Assumes all task ids are unique in this window.')
    parser.add_argument('start', metavar='S', type=task_query.mkdatetime, nargs='?', 
                   help='Start datetime of window for routines. Formatted "d/m/y H:M" e.g. "06/07/14 06:38"')
    
    parser.add_argument('end', metavar='E', type=task_query.mkdatetime, nargs='?', 
                   help='End datetime of window for routines. Formatted "d/m/y H:M" e.g. "06/07/14 06:38"')

    parser.add_argument('-t', '--tasks', type=int, default=1, nargs='?',
                    help='Number of tasks required for a routine to be considered')

    parser.add_argument('-ds', '--daily_start', type=task_query.mktime, nargs='?', default=time(0,0),
                    help='Daily start time of the routine. Formatted "H:M" e.g. "06:38". Default 00:00')

    parser.add_argument('-de', '--daily_end', type=task_query.mktime, nargs='?', default=time(23,59),
                    help='Daily end time of the routine. Formatted "H:M" e.g. "17:45". Default 23:59')

    
    args = parser.parse_args()

    assert args.daily_end > args.daily_start
     
    try:

        results = task_query.query_tasks(msg_store, event=[TaskEvent.ROUTINE_STARTED, TaskEvent.ROUTINE_STOPPED])

        # make sure we start with a ROUTINE_STARTED
        while results[0][0].event == TaskEvent.ROUTINE_STOPPED:
            del results[0]
            print 'pruned stop start'

        routines = []
        start = None
       
        # make sure we just have the pairs which are start/end
        for e in results:
            event = e[0]
            if event.event == TaskEvent.ROUTINE_STARTED:
                start = event
            elif event.event == TaskEvent.ROUTINE_STOPPED:
                assert start is not None
                routines.append((start, event))
                start = None

        allow_open = True
        if results[-1][0].event == TaskEvent.ROUTINE_STARTED and allow_open:
            dummy_end = TaskEvent(event = TaskEvent.ROUTINE_STOPPED, task = Task(), time = rospy.get_rostime())
            routines.append((results[-1][0], dummy_end))


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

        RoutineAnalyser(msg_store, filtered_routines, daily_start = args.daily_start, daily_end = args.daily_end).cmdloop()

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


        
