#!/usr/bin/env python
PKG = 'task_executor'
NAME = 'time_tester'

import rospy
import unittest
import rostest
import random

import actionlib
from strands_executive_msgs.msg import Task
from task_executor.msg import *
from task_executor.utils import CheckTaskActionServer, rostime_to_python
from Queue import Queue
from random import randrange
from mongodb_store.message_store import MessageStoreProxy
from geometry_msgs.msg import Pose, Point, Quaternion
from strands_executive_msgs import task_utils
from strands_executive_msgs.msg import Task
from strands_executive_msgs.srv import AddTasks, SetExecutionStatus
from topological_navigation.msg import GotoNodeAction
from strands_navigation_msgs.msg import TopologicalMap
from strands_navigation_msgs.srv import EstimateTravelTime
from std_msgs.msg import String
        
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


# class TestTaskTimings(object):
class TestTaskTimings(unittest.TestCase):

    def __init__(self, *args): 
        super(TestTaskTimings, self).__init__(*args)    
        rospy.init_node(NAME)        
        self.current_node = None

        rospy.Subscriber('/current_node', String, self.update_topological_location, queue_size=2)
        while self.current_node is None and not rospy.is_shutdown():
            rospy.sleep(0.5)


    def update_topological_location(self, node_name):
        self.current_node = node_name.data
        
    def test_single_task_timing(self):

        rospy.logwarn('test_single_task_timing')

        msg_store = MessageStoreProxy() 

        # now wait for the distance service
        time_srv_name = 'topological_navigation/travel_time_estimator'
        rospy.wait_for_service(time_srv_name, timeout=10)
        time_srv = rospy.ServiceProxy(time_srv_name, EstimateTravelTime)

        checks = []

        def checked(bool):
            checks.append(bool)

        task_checker = CheckTaskActionServer(result_fn = checked)
        task_checker.start()

        now = rospy.get_rostime()
        delay = rospy.Duration(5)

        current_node = self.current_node
        

        task = Task()
        task.action = 'check_task'
        task.start_node_id = 'v_1'
        # make sure action dur
        travel_time = time_srv(current_node, task.start_node_id).travel_time
        task.max_duration = rospy.Duration(travel_time.to_sec()/4)
        task.start_after = now + delay + travel_time
        task.end_before = task.start_after + rospy.Duration(travel_time.to_sec()/2)

        # add the task argument which is a reference to a copy of itself in the db
        object_id = msg_store.insert(task)
        task_utils.add_object_id_argument(task, object_id, Task)

        client = actionlib.SimpleActionClient('check_task', TaskTestAction)
        client.wait_for_server()


        # get services to call into execution framework
        add_tasks, set_execution_status = get_services()



        add_tasks([task])
        set_execution_status(True)

        while len(checks) < 1 and not rospy.is_shutdown():
            rospy.sleep(0.1)

        for result in checks:
            self.assertTrue(result)


# if __name__ == '__main__':
#     # rostest.rosrun(PKG, NAME, TestEntry, sys.argv)
#     rospy.init_node(NAME)
#     executor = TestTaskTimings()        
#     executor.execute()

#     rospy.spin()

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestTaskTimings, sys.argv)


