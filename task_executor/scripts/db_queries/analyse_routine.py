#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from strands_executive_msgs.msg import Task, TaskEvent
from datetime import datetime, timedelta, time, date
from task_executor import task_routine, task_query
from task_executor.utils import rostime_to_python, python_to_rostime
from task_executor.task_query import task_groups_in_window, daily_windows_in_range, reconstruct_routines
from task_executor.routine_analyser import RoutineAnalyser

import pytz
from dateutil.relativedelta import *
import matplotlib.patches as mpatches
import numpy as np
import matplotlib.pyplot as plt

import argparse
import cmd

def init():
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
        tz = pytz.timezone(pytz.country_timezones[args.time_zone][0])

        analysis_start = args.start
        analysis_end = args.end

        if analysis_start is not None:
            analysis_start = analysis_start.replace(tzinfo=tz)
        
        if analysis_end is not None:
            analysis_end = analysis_end.replace(tzinfo=tz)

        if analysis_end is not None and analysis_start is not None:
            assert analysis_end > analysis_start


        filtered_routines = reconstruct_routines(task_query.query_tasks(msg_store, start_date=analysis_start, end_date=analysis_end))

        


        analyser = RoutineAnalyser(msg_store, filtered_routines, daily_start = args.daily_start.replace(tzinfo=tz), daily_end = args.daily_end.replace(tzinfo=tz), tz=tz)
        # interactive mode
        analyser.cmdloop()




    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == '__main__':
    init()
    


        
