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


if __name__ == '__main__':

    rospy.init_node('tutorial_2')

    # construct task
    task = Task()

    # set action to execute
    task.action = '/wait_action'

    # set how long this should be allowed to run in the worst case
    max_wait_minutes = 60
    task.max_duration = rospy.Duration(max_wait_minutes)

    # add arguments in the order they are defined in the action definition

    # specify a wait of 10 seconds
    task_utils.add_time_argument(task, rospy.Time())
    task_utils.add_duration_argument(task, rospy.Duration(10))

    # don't start the task until at least 10 seconds in the future
    task.start_after = rospy.get_rostime() + rospy.Duration(10)
    # and give a window of three times the max execution time in which to execute
    task.end_before = task.start_after + rospy.Duration(task.max_duration.to_sec() * 3)

    # get the services we need to interact with the framework
    set_execution_status = get_execution_status_service()
    add_tasks = get_add_tasks_service()

    # if this is the first time tasks have been added to the to framework since it was started
    # you need to enable execution
    set_execution_status(True)

    add_tasks([task])
