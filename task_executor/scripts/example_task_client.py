#!/usr/bin/env python

import rospy
import mongodb_store_msgs.srv as dc_srv
from mongodb_store_msgs.msg import StringPair
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from geometry_msgs.msg import Pose, Point, Quaternion
import StringIO
import random

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

def publish_topological_map():
    """
    Publish a topological map for testing.
    """
    # create a test topological map
    width = 5 
    height = 5 
    nodeSeparation = 10.0

    test_nodes = topological_navigation.testing.create_cross_map(width = width, height = height, nodeSeparation = nodeSeparation)

    # now insert the map into the database
    msg_store = MessageStoreProxy(collection='topological_maps')

    map_name = 'test_top_map'

    meta = {}
    meta['map'] = map_name
    meta['pointset'] = map_name

    for (nodeName, node) in test_nodes.iteritems():
        meta["node"] = nodeName
        node.map = meta['map']
        node.pointset = meta['pointset']
        msg_store.insert(node,meta)

    # and publish the map
    ps = map_publisher(map_name)

    return test_nodes
    


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
    actual_action_duration = rospy.Duration(20)
    actual_drive_duration = rospy.Duration(1)

    # create a task we will copy later,
    master_task = create_master_task()


    # now create a bunch of task with different times
    task_count = 1
    start_of_window = rospy.get_rostime() 
    # max time we will tell teh scheduler any action is expected to run for
    max_action_duration = actual_drive_duration + actual_action_duration + actual_action_duration   
    # a total time window to fit all the tasks in
    end_of_window = start_of_window + rospy.Duration(max_action_duration.secs * (task_count + 1))

    # how long we tell the scheduler we'll run for
    # this one will be normal
    # scheduled_duration = actual_action_duration 
    # this one will trigger task preemption
    scheduled_duration = rospy.Duration(actual_action_duration.secs / 4)


    test_nodes = publish_topological_map()

    tasks = []
    # create individual tasks with differnt duration to happen within the window
    for task_id in range(task_count):    
        # copy the task from the master
        timed_task = deepcopy(master_task)
        task_node = random.choice(test_nodes).name
        timed_task.start_node_id=task_node
        timed_task.end_node_id=task_node
        timed_task.start_after = start_of_window
        timed_task.end_before = end_of_window
        # tell the scheduler we might take longer than we think
        timed_task.max_duration = scheduled_duration
        tasks.append(timed_task)

        # this provides windows that need execution delays
        # start_of_window += max_action_duration


    # get services to call into execution framework
    add_tasks, set_execution_status = get_services()


    # register task with the scheduler
    task_ids = add_tasks(tasks)
    print "Added %s" % task_ids


    if False:
        action_server = TestTaskAction(expected_action_duration=actual_action_duration, expected_drive_duration=actual_drive_duration)


    # Set the task executor is running
    set_execution_status(True)



    # rospy.sleep(actual_action_duration)

    # # # now create a second bunch of tasks which should not be possible
    # task_ids = add_tasks(tasks)
    # print "Added %s" % task_ids

    print 'spinning'
    rospy.spin()
