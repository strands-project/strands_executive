#!/usr/bin/env python

import rospy

# for the task message
from strands_executive_msgs.msg import Task
# for the helper functions which add arguments to task messages
from strands_executive_msgs import task_utils
# service types for interacting with executive framework
from strands_executive_msgs.srv import AddTasks, DemandTask, SetExecutionStatus


def get_service(service_name, service_type):    
    rospy.loginfo('Waiting for %s service...' % service_name)
    rospy.wait_for_service(service_name)
    rospy.loginfo("Done")        
    return rospy.ServiceProxy(service_name, service_type)

def get_execution_status_service():
    return get_service('/task_executor/set_execution_status', SetExecutionStatus)

def get_add_tasks_service():
    return get_service('/task_executor/add_tasks', AddTasks)

def wait_task_at_waypoint(wp, wait_duration, start_after, end_before):
    # construct task
    task = Task()

    # set action to execute
    task.action = '/wait_action'

    # set how long this should be allowed to run in the worst case
    max_wait_minutes = 60
    task.max_duration = rospy.Duration(max_wait_minutes)

    task_utils.add_time_argument(task, rospy.Time())
    task_utils.add_duration_argument(task, rospy.Duration(wait_duration))

    task.start_after = start_after
    task.end_before = end_before

    task.start_node_id = wp
    task.end_node_id = wp

    return task

if __name__ == '__main__':

    rospy.init_node('tutorial_3')

    # get the services we need to interact with the framework
    set_execution_status = get_execution_status_service()
    add_tasks = get_add_tasks_service()

    # if this is the first time tasks have been added to the to framework since it was started
    # you need to enable execution
    set_execution_status(True)

    wps = ['WayPoint6', 'WayPoint3', 'WayPoint5', 'WayPoint3',]

    wait_secs = 10
    assumed_travel = 300
    tasks = [wait_task_at_waypoint(wp, wait_secs, rospy.get_rostime(), rospy.get_rostime() + rospy.Duration(wait_secs * len(wps) + assumed_travel * len(wps))) for wp in wps]

    add_tasks(tasks)



