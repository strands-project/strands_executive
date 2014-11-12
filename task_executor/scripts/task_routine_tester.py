#!/usr/bin/env python

import rospy

from datetime import datetime, timedelta, time
from dateutil.tz import tzlocal


import mongodb_store_msgs.srv as dc_srv
from mongodb_store_msgs.msg import StringPair
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from geometry_msgs.msg import Pose, Point, Quaternion
import StringIO

from strands_executive_msgs import task_utils
from strands_executive_msgs.msg import Task
from strands_executive_msgs.srv import AddTasks, SetExecutionStatus
# import strands_executive_msgs

import rospy
import actionlib
from strands_executive_msgs.msg import Task
from task_executor.msg import *
from topological_navigation.msg import GotoNodeAction
from copy import deepcopy
from random import random


from task_executor import task_routine
from task_executor.utils import TestTaskAction


def get_services():
    # get services necessary to do the jon
    add_tasks_srv_name = '/task_executor/add_tasks'
    set_exe_stat_srv_name = '/task_executor/set_execution_status'
    rospy.loginfo("Waiting for task_executor service...")
    rospy.wait_for_service(add_tasks_srv_name)
    rospy.wait_for_service(set_exe_stat_srv_name)
    rospy.loginfo("Done")        
    add_tasks_srv = rospy.ServiceProxy(add_tasks_srv_name, AddTasks)
    set_execution_status = rospy.ServiceProxy(set_exe_stat_srv_name, SetExecutionStatus)
    return add_tasks_srv, set_execution_status

def create_wait_task(max_duration):
    master_task = Task(action='wait_action',start_node_id='ChargingPoint', max_duration=max_duration)
    task_utils.add_time_argument(master_task, rospy.Time())
    task_utils.add_duration_argument(master_task, max_duration)
    return master_task

def on_day_start():
    rospy.loginfo('on_day_start')


def on_day_end():
    rospy.loginfo('on_day_end')

        
if __name__ == '__main__':
    rospy.init_node("task_routine_tester")
     # wait for simulated time to kick in as rospy.get_rostime() is 0 until first clock message received
    while not rospy.is_shutdown() and rospy.get_rostime().secs == 0:
        pass

    # create a task we will copy later
    actual_action_duration = rospy.Duration(20)
    task = create_wait_task(actual_action_duration)

    # get services to call into execution framework
    add_tasks, set_execution_status = get_services()
    set_execution_status(True)

    # some useful times
    localtz = tzlocal()
    # the time the robot will be active
    start = time(8,45, tzinfo=localtz)
    end = time(23,15, tzinfo=localtz)
    midday = time(12,00, tzinfo=localtz)

    morning = (start, midday)
    afternoon = (midday, end)

    routine = task_routine.DailyRoutine(start, end)
   
    routine.repeat_every_mins(task, times=1)

    # create the object which will talk to the scheduler
    runner = task_routine.DailyRoutineRunner(start, end, add_tasks, day_start_cb=on_day_start, day_end_cb=on_day_end)
    # pass the routine tasks on to the runner which handles the daily instantiation of actual tasks
    runner.add_tasks(routine.get_routine_tasks())

    rospy.spin()
