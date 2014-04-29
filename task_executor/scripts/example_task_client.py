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
from topological_navigation.msg import GotoNodeAction, GotoNodeResult
from copy import deepcopy
from random import random


class TestTaskAction(object):
    """ 
    Creates the action servers the example tasks required.
    """
    def __init__(self, expected_action_duration=1, expected_drive_duration=1):
        self.expected_action_duration = expected_action_duration
        self.expected_drive_duration = expected_drive_duration
        self.result   =  GotoNodeResult()
        self.nav_server = actionlib.SimpleActionServer('topological_navigation', GotoNodeAction, execute_cb = self.nav_callback, auto_start = False)
        self.nav_server.start() 
        self.task_server = actionlib.SimpleActionServer('test_task', TestExecutionAction, execute_cb = self.execute, auto_start = False)
        self.task_server.start() 
        

    def execute(self, goal):
        print 'called with goal %s'%goal.some_goal_string
        rospy.sleep(self.expected_action_duration)                
        self.task_server.set_succeeded()
        # self.task_server.set_aborted()
        # print 'done here'
        

    def nav_callback(self, goal):
        print 'called with nav goal %s'%goal.target
        rospy.sleep(self.expected_drive_duration)  
        self.result.success=True
        self.nav_server.set_succeeded(self.result)
        # self.nav_server.set_aborted(self.result)
        # print 'done nav'

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
    rospy.init_node("example_task_client")

    # Perform my own actions
    actual_action_duration = rospy.Duration(10)
    actual_drive_duration = rospy.Duration(1)

    # create a task we will copy later,
    master_task = create_master_task()

    # get services to call into execution framework
    add_tasks, set_execution_status = get_services()

    # now create a bunch of task with different times
    task_count = 3
    start_of_window = rospy.get_rostime() 
    # max time we will tell teh scheduler any action is expected to run for
    max_action_duration = actual_drive_duration + actual_action_duration + actual_action_duration   
    # a total time window to fit all the tasks in
    end_of_window = start_of_window + rospy.Duration(max_action_duration.secs * (task_count + 1))


    tasks = []
    # create individual tasks with differnt duration to happen within the window
    for task_id in range(task_count):    
        # copy the task from the master
        timed_task = deepcopy(master_task)
        timed_task.start_node_id=str(task_id)
        timed_task.end_node_id=str(task_id)
        timed_task.start_after = start_of_window
        timed_task.end_before = end_of_window
        # tell the scheduler we might take longer than we think
        timed_task.max_duration = actual_action_duration # + rospy.Duration(actual_action_duration.secs * random())
        tasks.append(timed_task)

        # this provides windows that need execution delays
        # start_of_window += max_action_duration

    # register task with the scheduler
    task_ids = add_tasks(tasks)
    print "Added %s" % task_ids


    if True:
        action_server = TestTaskAction(expected_action_duration=actual_action_duration, expected_drive_duration=actual_drive_duration)


    # Set the task executor is running
    set_execution_status(True)



    # rospy.sleep(actual_action_duration)

    # # # now create a second bunch of tasks which should not be possible
    # task_ids = add_tasks(tasks)
    # print "Added %s" % task_ids

    print 'spinning'
    rospy.spin()
