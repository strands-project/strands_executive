#!/usr/bin/env python

import rospy
from strands_executive_msgs import task_utils
from strands_executive_msgs.msg import Task
from strands_executive_msgs.srv import AddTasks, SetExecutionStatus
from strands_navigation_msgs.msg import *
from rosgraph_msgs.msg import Clock
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


if __name__ == '__main__':
    rospy.init_node("example_add_client")


    if rospy.get_param('use_sim_time', False):
        rospy.loginfo('Using sim time, waiting for time update')
        rospy.wait_for_message('clock', Clock)


    # get services to call into execution framework
    add_task, set_execution_status = get_services()

    print 'Requesting wait at %s for %s seconds' % (sys.argv[1], sys.argv[2])

    max_duration = rospy.Duration(int(sys.argv[2]))
    wait_task = Task(action='wait_action',start_node_id=sys.argv[1], max_duration=max_duration)

    wait_task.start_after = rospy.get_rostime() 
    wait_task.end_before = wait_task.start_after + max_duration + max_duration + rospy.Duration(1200)

    task_utils.add_time_argument(wait_task, rospy.Time())
    task_utils.add_duration_argument(wait_task, max_duration)
    

    # wait_task = Task(action='',start_node_id=sys.argv[1], max_duration=max_duration)

    task_id = add_task([wait_task])
    
    # Set the task executor running (if it isn't already)
    set_execution_status(True)

    # now let's stop execution while it's going on
    # rospy.sleep(int(sys.argv[2])/2)
    # set_execution_status(False)