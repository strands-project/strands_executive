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
from task_executor.task_query import task_groups_in_window, daily_windows_in_range
import pytz
from dateutil.relativedelta import *

import numpy as np
import matplotlib.pyplot as plt

import argparse
import cmd


class RoutineAnalyser(cmd.Cmd):

    def __init__(self, msg_store, routine_pairs, tz, daily_start = None, daily_end = None):
        cmd.Cmd.__init__(self)
        # super(RoutineAnalyser, self).__init__()
        self.routine_pairs = routine_pairs
        self.msg_store = msg_store
        self.daily_start = daily_start if daily_start is not None else time(0,0)
        self.daily_end = daily_end if daily_end is not None else time(23,59)
        # hand code for now
        self.days_off = ['Saturday', 'Sunday', date(2015, 5, 25), date(2015, 5, 4) ]
        # self.days_off = []
        self.tz = tz
        self.colour_mappings = dict()


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
            start = rostime_to_python(self.routine_pairs[i][0].time, self.tz)
            end = rostime_to_python(self.routine_pairs[i][1].time, self.tz)
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


            window_start = rostime_to_python(self.routine_pairs[idx][0].time, self.tz)
            window_end = rostime_to_python(self.routine_pairs[idx][1].time, self.tz)

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
            print '%s: %s from %s -> %s' % (daily_start.date(), autonomy_duration, day_duration, autonomy_percentage)
            return autonomy_duration, day_duration            

        else:
            print 'Skipping %s %s' % (day, date)
            return timedelta(), timedelta()



    def do_autonomy(self, idx):    
        try:
            idx = int(idx)

            if not self.check_idx(idx):
                return

            window_start = rostime_to_python(self.routine_pairs[idx][0].time, self.tz)
            window_end = rostime_to_python(self.routine_pairs[idx][1].time, self.tz)


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

    def do_timeplot(self, idx):
        try: 
            idx = int(idx)

            if not self.check_idx(idx):
                return

            window_start = rostime_to_python(self.routine_pairs[idx][0].time, self.tz)
            window_end = rostime_to_python(self.routine_pairs[idx][1].time, self.tz)

            # get all the task starts
            results = task_query.query_tasks(self.msg_store, 
                        event=TaskEvent.TASK_STARTED, 
                        start_date=window_start,
                        end_date=window_end,
                        )

            # convert to an array of times
            dates = [rostime_to_python(event[0].time, self.tz) for event in results]            
            n, bins, patches = plt.hist([date.hour + date.minute/60.0 for date in dates], bins = 24*60/15)
            plt.show()

        except ValueError, e:
            print 'provided argument was not an int: %s' % idx



    def do_lateness(self, idx):
        """ Not recommended for use...
        """
        try: 
            idx = int(idx)

            if not self.check_idx(idx):
                return

            window_start = rostime_to_python(self.routine_pairs[idx][0].time, self.tz)
            window_end = rostime_to_python(self.routine_pairs[idx][1].time, self.tz)

            means = []
            stddev = []
            count = []


            for daily_start, daily_end in daily_windows_in_range(self.daily_start, self.daily_end, window_start, window_end):
                day_errors = np.array([((task_group[2].task.execution_time - task_group[2].time).to_sec()/(task_group[2].time - task_group[1].time).to_sec()) for task_group in task_groups_in_window(daily_start, daily_end, self.msg_store, event=[TaskEvent.ADDED, TaskEvent.NAVIGATION_STARTED, TaskEvent.NAVIGATION_SUCCEEDED])])        
                if len(day_errors) > 5:
                    means.append(day_errors.mean())
                    stddev.append(day_errors.std())
                    count.append(len(means))

            plt.errorbar(count, means, stddev)

            plt.show()


        except ValueError, e:
            print 'provided argument was not an int: %s' % idx




    def draw_task(self, y, action, start_time, end_time):
        # start and end times are ros times, we need to turn them into just hours
            
        start_midnight = datetime.fromordinal(rostime_to_python(start_time, self.tz).date().toordinal())
        start_midnight = start_midnight.replace(tzinfo=self.tz)

        start = (rostime_to_python(start_time, self.tz) - start_midnight).total_seconds()
        end = (rostime_to_python(end_time, self.tz) - start_midnight).total_seconds()

        if action not in self.colour_mappings:
            colour_map = plt.get_cmap('Paired')    
            self.colour_mappings[action] = colour_map(len(self.colour_mappings) * 30)

        plt.hlines(y, start, end, self.colour_mappings[action], lw=10)

    def do_taskplot(self, idx):
        try: 
            idx = int(idx)

            if not self.check_idx(idx):
                return

            window_start = rostime_to_python(self.routine_pairs[idx][0].time, self.tz)
            window_end = rostime_to_python(self.routine_pairs[idx][1].time, self.tz)

            daily_tasks = []
            for daily_start, daily_end in daily_windows_in_range(self.daily_start, self.daily_end, window_start, window_end, self.days_off):
                
                succeeded_tasks = np.array([(task_group[0].task.action, task_group[0].time, task_group[1].time) for task_group in task_groups_in_window(daily_start, daily_end, self.msg_store, event=[TaskEvent.TASK_STARTED, TaskEvent.TASK_SUCCEEDED])], dtype=object)
                failed_tasks = np.array([(task_group[0].task.action, task_group[0].time, task_group[1].time) for task_group in task_groups_in_window(daily_start, daily_end, self.msg_store, event=[TaskEvent.TASK_STARTED, TaskEvent.TASK_FAILED])], dtype=object)        
                # print len(succeeded_tasks)
                # print len(failed_tasks)

                # can't concatenate 0  length np.arrays
                if len(succeeded_tasks) == 0:
                    all_tasks = failed_tasks
                elif len(failed_tasks) == 0:
                    # print succeeded_tasks
                    all_tasks = succeeded_tasks
                else:
                    all_tasks = np.concatenate((succeeded_tasks,failed_tasks), axis=0)
                
                print daily_start.date(), len(all_tasks) 
                daily_tasks.append([daily_start.date(), all_tasks])

            y_sep = 10
            y = 0

            # 
            y_label_points = []
            y_labels = []

            for task_date, task_times in reversed(daily_tasks):
                y += y_sep
                y_label_points.append(y)
                y_labels.append(task_date.strftime('%A, %B %d %Y'))
                for task_time in task_times:
                    self.draw_task(y, task_time[0], task_time[1], task_time[2])        
                
                
            plt.ylim(0, y + y_sep)

            x_label_points = []
            x_labels = []
            for hour in range(self.daily_start.hour-1, self.daily_end.hour+1):
                seconds_per_hour = 60 * 60
                x_label_points.append(hour * seconds_per_hour)
                x_labels.append('%s:00' % hour)

            plt.xticks(x_label_points, x_labels, rotation='vertical')
            plt.yticks(y_label_points, y_labels, rotation='horizontal')

            # (y, xstart, xstop, color, lw=4)
            # plt.vlines(xstart, y+0.03, y-0.03, color, lw=2)
            # plt.vlines(xstop, y+0.03, y-0.03, color, lw=2)

            #Setup the plot
            # ax = plt.gca()
            # ax.xaxis_date()
            # # myFmt = DateFormatter('%H:%M:%S')
            # ax.xaxis.set_major_formatter(myFmt)
            # ax.xaxis.set_major_locator(SecondLocator(interval=20)) # used to be SecondLocator(0, interval=20)

            # #To adjust the xlimits a timedelta is needed.
            # delta = (stop.max() - start.min())/10

            # plt.yticks(y[unique_idx], captions)
            # plt.ylim(0,1)
            # plt.xlim(start.min()-delta, stop.max()+delta)
            # plt.xlabel('Time')
            plt.show()


                
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
            window_start = rostime_to_python(self.routine_pairs[idx][0].time, self.tz)
            window_end = rostime_to_python(self.routine_pairs[idx][1].time, self.tz)

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

    parser.add_argument('-tz', '--time_zone', type=str, nargs='?', default='gb',
                    help='Country code for timezone lookup. Default is "gb". Examples include "at" and "de".')
    
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
            else:
                print 'unterminated routine'

        allow_open = True
        if results[-1][0].event == TaskEvent.ROUTINE_STARTED and allow_open:
            dummy_end = TaskEvent(event = TaskEvent.ROUTINE_STOPPED, task = Task(), time = rospy.get_rostime())
            routines.append((results[-1][0], dummy_end))

        tz = pytz.timezone(pytz.country_timezones[args.time_zone][0])


        filtered_routines = []
        for i in range(len(routines)):
            start = rostime_to_python(routines[i][0].time, tz)
            end = rostime_to_python(routines[i][1].time, tz)
            results = task_query.query_tasks(msg_store, 
                        start_date=start,
                        end_date=end,
                        event=[TaskEvent.TASK_STARTED]
                        )

            if len(results) >= args.tasks:
                filtered_routines.append(routines[i])
                # print 'routine %s: %s to %s, duration: %s, tasks: %s' % (len(filtered_routines)-1, start, end, end-start, len(results))


        analyser = RoutineAnalyser(msg_store, filtered_routines, daily_start = args.daily_start.replace(tzinfo=tz), daily_end = args.daily_end.replace(tzinfo=tz), tz=tz)
        # interactive mode
        # analyser.cmdloop()
        # testing some things
        analyser.do_merge('all')
        # analyser.do_timeplot('0')

        # analyser.do_merge('21')
        # analyser.do_merge('21')
        # analyser.do_merge('21')
        # analyser.do_lateness('21')
        analyser.do_taskplot('0')
        # analyser.do_summarise('20')
        # analyser.do_print('20')





    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


        
