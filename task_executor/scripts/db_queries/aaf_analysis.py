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
    rospy.init_node("g4s_analysis")

    msg_store = MessageStoreProxy(collection='task_events')

    try:
        tz = pytz.timezone(pytz.country_timezones['at'][0])


        analysis_start = datetime(2015,5,18,8,0,tzinfo=tz)
        analysis_end = datetime(2015,6,17,17,0,tzinfo=tz)

        daily_start = time(8,00, tzinfo=tz)
        daily_end = time(21,00, tzinfo=tz)

        filtered_routines = reconstruct_routines(task_query.query_tasks(msg_store, start_date=analysis_start, end_date=analysis_end))

        days_off = [date(2015,5, 23),
            date(2015,5, 24),
            date(2015,5, 25),
            date(2015,5, 30),
            date(2015,5, 31),
            date(2015, 6, 4),
            date(2015, 6, 6),
            date(2015, 6, 7),
            date(2015,6, 13),
            date(2015,6, 14)]

        analyser = RoutineAnalyser(msg_store, filtered_routines, daily_start = daily_start, daily_end = daily_end, tz=tz, days_off = days_off)
 
        analyser.do_print('')
        analyser.do_days('3')
        analyser.do_merge('all')
        analyser.do_print('')
        analyser.do_days('0')
        analyser.do_autonomy('0')
        analyser.do_summarise('0')
        analyser.do_taskplot('0 aaf')
        analyser.do_timeplot('0 aaf')


    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == '__main__':
    init()
    


        
