#!/usr/bin/env python

import rospy
from strands_executive_msgs import task_utils
from strands_executive_msgs.msg import Task
from strands_executive_msgs.srv import AddTasks, SetExecutionStatus
from strands_navigation_msgs.msg import *
import sys

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


def create_wait_task(node, secs=rospy.Duration(10)):
    wait_task = Task(action='wait_action',start_node_id=node, end_node_id=node, max_duration=secs)
    task_utils.add_time_argument(wait_task, rospy.Time())
    task_utils.add_duration_argument(wait_task, secs)
    return wait_task

if __name__ == '__main__':
    rospy.init_node("example_multi_add_client")

    # get services to call into execution framework
    add_task, set_execution_status = get_services()

    nodes = ['WayPoint1', 'WayPoint2', 'ChargingPoint']    
    tasks = map(create_wait_task, nodes)

    # wait_task = Task(action='',start_node_id=sys.argv[1], max_duration=max_duration)

    task_id = add_task(tasks)
    
    # Set the task executor running (if it isn't already)
    resp = set_execution_status(True)

    # now let's stop execution while it's going on
    # rospy.sleep(4)
    # resp = set_execution_status(False)
    # rospy.loginfo('Success: %s' % resp.success)
    # rospy.loginfo('Wait: %s' % resp.remaining_execution_time)

    # # and start again
    # rospy.sleep(2)
    # set_execution_status(True)
