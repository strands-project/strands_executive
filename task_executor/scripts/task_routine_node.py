#!/usr/bin/env python

import rospy
import actionlib
from strands_executive_msgs.msg import Task
from strands_executive_msgs import task_utils
from task_executor.msg import *
from topological_navigation.msg import GotoNodeAction
from task_executor import task_routine
from datetime import *
from threading import Thread
import mongodb_store_msgs.srv as dc_srv
from mongodb_store_msgs.msg import StringPair
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from geometry_msgs.msg import Pose, Point, Quaternion
from dateutil.tz import tzlocal

        

def dummy_task():
    """ 
    Create an example of a task which we'll copy for other tasks later.
    This is a good example of creating a task with a variety of arguments.
    """
     # need message store to pass objects around
    msg_store = MessageStoreProxy() 

    # get the pose of a named object
    pose_name = "my favourite pose"

    # get the pose if it's there
    message, meta =  msg_store.query_named(pose_name, Pose._type)
    # if it's not there, add it in
    if message == None: 
        message = Pose(Point(0, 1, 2), Quaternion(3, 4,  5, 6))
        pose_id = msg_store.insert_named(pose_name, message)
    else:
        pose_id = meta["_id"]           

    master_task = Task(action='test_task')        
    task_utils.add_string_argument(master_task, 'hello world')
    task_utils.add_object_id_argument(master_task, pose_id, Pose)
    task_utils.add_int_argument(master_task, 24)
    task_utils.add_float_argument(master_task, 63.678)
    return master_task

        

if __name__ == '__main__':

    rospy.init_node("task_routine", log_level=rospy.INFO)
 
    # wait for simulated time to kick in as rospy.get_rostime() is 0 until first clock message received
    while not rospy.is_shutdown() and rospy.get_rostime().secs == 0:
        pass

    localtz = tzlocal()

    start = time(8,30, tzinfo=localtz)
    ten = time(10,00, tzinfo=localtz)
    midday = time(12,00, tzinfo=localtz)
    end = time(17,00, tzinfo=localtz)
    morning = (start, midday)
    afternoon = (midday, end)

    task = dummy_task()

    routine = task_routine.DailyRoutine(start, end)
    routine.repeat_every_hour(task, hours=2)
    
    # all these should except
    # routine.add_tasks([task], midday, start)
    # routine.add_tasks([task], start, start)
    # task.max_duration = 60 * 60 * 12;
    # routine.add_tasks([task], start, midday)


    # routine = task_routine.DailyRoutineRunner(start, end)
    # task.max_duration = rospy.Duration(timedelta(seconds=60).total_seconds());
    # routine.add_tasks([([task, task], morning), ([task, task], afternoon)])

    rospy.spin()

