#!/usr/bin/env python

import rospy
from strands_executive_msgs import task_utils
from strands_executive_msgs.msg import Task
from strands_executive_msgs.srv import AddTasks, SetExecutionStatus
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
    set_execution_status = \
        rospy.ServiceProxy(
            set_exe_stat_srv_name,
            SetExecutionStatus)
    return add_tasks_srv, set_execution_status


if __name__ == '__main__':
    rospy.init_node("example_add_time_critical_client")

    # get services to call into execution framework
    add_task, set_execution_status = get_services()

    wp = 'WayPoint2'
    duration_secs = 30
    wait_before = rospy.Duration(30)

    max_duration = rospy.Duration(duration_secs)
    wait_task = Task(action='wait_action',
                     start_node_id=wp, max_duration=max_duration)
    wait_task.start_after = rospy.get_rostime() + wait_before
    wait_task.end_before = wait_task.start_after
    task_utils.add_time_argument(wait_task, rospy.Time())
    task_utils.add_duration_argument(wait_task, max_duration)

    task_id = add_task([wait_task])

    # Set the task executor running (if it isn't already)
    set_execution_status(True)

    # now let's stop execution while it's going on
    # rospy.sleep(int(sys.argv[2])/2)
    # set_execution_status(False)
