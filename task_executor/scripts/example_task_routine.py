#!/usr/bin/env python

import rospy
import ros_datacentre_msgs.srv as dc_srv
from ros_datacentre_msgs.msg import StringPair
import ros_datacentre.util as dc_util
from ros_datacentre.message_store import MessageStoreProxy
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
from dateutil.tz import *
from datetime import *
from task_executor import task_routine



class TestTaskAction(object):
    """ 
    Creates the action servers the example tasks required.
    """
    def __init__(self, expected_action_duration=1, expected_drive_duration=1):
        self.expected_action_duration = expected_action_duration
        self.expected_drive_duration = expected_drive_duration
        self.nav_server = actionlib.SimpleActionServer('topological_navigation', GotoNodeAction, execute_cb = self.nav_callback, auto_start = False)
        self.nav_server.start() 
        self.task_server = actionlib.SimpleActionServer('test_task', TestExecutionAction, execute_cb = self.execute, auto_start = False)
        self.task_server.start() 
        

    def execute(self, goal):
        print 'called with goal %s'%goal
        rospy.sleep(self.expected_action_duration)
        print 'done here'
        self.task_server.set_succeeded()

    def nav_callback(self, goal):
        print 'called with nav goal %s'%goal
        rospy.sleep(self.expected_drive_duration)
        print 'done nav'
        self.nav_server.set_succeeded()

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

def create_master_task():
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
    rospy.init_node("example_task_routine")
     # wait for simulated time to kick in as rospy.get_rostime() is 0 until first clock message received
    while not rospy.is_shutdown() and rospy.get_rostime().secs == 0:
        pass


    # Perform my own actions
    actual_action_duration = rospy.Duration(4)
    if True:
        action_server = TestTaskAction(expected_action_duration=actual_action_duration)


    # create a task we will copy later
    task = create_master_task()

    # get services to call into execution framework
    add_tasks, set_execution_status = get_services()


    # some useful times
    localtz = tzlocal()
    # the time the robot will be active
    start = time(8,30, tzinfo=localtz)
    end = time(17,00, tzinfo=localtz)
    midday = time(12,00, tzinfo=localtz)

    morning = (start, midday)
    afternoon = (midday, end)

    routine = task_routine.DailyRoutine(start, end)
    # do this task every day
    routine.repeat_every_day(task)
    # and every two hours during the day
    routine.repeat_every_hour(task, hours=2)
    # once in the morning
    routine.repeat_every(task, *morning)
    # and twice in the afternoon
    routine.repeat_every(task, *afternoon, times=2)

    # create the object which will talk to the scheduler
    runner = task_routine.DailyRoutineRunner(start, end, add_tasks)
    # pass the routine tasks on to the runner which handles the daily instantiation of actual tasks
    runner.add_tasks(routine.get_routine_tasks())

    # # Set the task executor is running
    set_execution_status(True)

    rospy.spin()
