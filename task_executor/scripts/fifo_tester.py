#!/usr/bin/env python
PKG = 'task_executor'
NAME = 'fifo_tester'

import rospy
import unittest
import rostest

import actionlib
from strands_executive_msgs.msg import Task
from task_executor.msg import *
from Queue import Queue
from random import randrange
from ros_datacentre.message_store import MessageStoreProxy
from geometry_msgs.msg import Pose, Point, Quaternion
from strands_executive_msgs import task_utils
from strands_executive_msgs.msg import Task
from strands_executive_msgs.srv import AddTask, SetExecutionStatus


class FakeActionServer(object):
    def __init__(self, action_string, action_sleep, master, tester):
        self.master = master
        self.tester = tester
        self.action_string = action_string
        self.action_sleep = action_sleep
        self.server = actionlib.SimpleActionServer(action_string, TestExecutionAction, self.execute, False)
        self.server.start() 

    def execute(self, goal):
        # print 'called with goal %s'%goal
        rospy.sleep(self.action_sleep)

        task_description = self.master.task_descriptions.pop(0)
        self.tester.assertEquals(task_description[1], self.action_string)
        self.tester.assertEquals(task_description[2], goal.some_goal_string)
        self.tester.assertEquals(task_description[3], goal.test_pose)
        self.server.set_succeeded()



class FIFOTester(object):
    def __init__(self, action_types, action_prefix, task_descriptions, action_sleep, tester):
        rospy.init_node(NAME)
        self.tester = tester
        self.task_descriptions = task_descriptions
        self.action_servers = [FakeActionServer(action_prefix + str(n), action_sleep, self, tester) for n in range(action_types)]

    def wait_for_completion(self, wait_duration):
        # give everything time to complete
        rospy.sleep(wait_duration)
        self.tester.assertEquals(self.task_descriptions, [])


class TestEntry(unittest.TestCase):

    def test_fifo_task_executor(self):   
        waypoints = 5
        action_types = 5
        test_tasks = 5
        action_sleep = rospy.Duration.from_sec(1)
        waypoint_prefix = 'waypoint_'
        action_prefix = 'test_task_'

        msg_store = MessageStoreProxy() 

        task_descriptions = []
        # list comprehension seemed to use the same result from randrange
        for n in range(test_tasks):
            string = 'oh what a lovely number %s is' % n
            pose = Pose(Point(n, 1, 2), Quaternion(3, 4,  5, 6))        
            task_descriptions += [[waypoint_prefix + str(randrange(waypoints)), 
                action_prefix + str(randrange(action_types)),
                string,
                pose]]
        assert test_tasks == len(task_descriptions)

        executor = FIFOTester(action_types, action_prefix, task_descriptions, action_sleep, self)    

        # get task services
        add_task_srv_name = '/task_executor/add_task'
        set_exe_stat_srv_name = '/task_executor/set_execution_status'
        rospy.loginfo("Waiting for task_executor service...")
        rospy.wait_for_service(add_task_srv_name)
        rospy.wait_for_service(set_exe_stat_srv_name)
        rospy.loginfo("Done")        
        add_task_srv = rospy.ServiceProxy(add_task_srv_name, AddTask)
        set_execution_status = rospy.ServiceProxy(set_exe_stat_srv_name, SetExecutionStatus)
        
        try:
                
            for task_description in task_descriptions:    
                # create the task from the description
                task = Task(node_id=task_description[0], action=task_description[1])        
                # add some dummy arguments
                task_utils.add_string_argument(task, task_description[2])
                task_utils.add_object_id_argument(task, msg_store.insert(task_description[3]), Pose)
                task_id = add_task_srv(task)
                print task_id
                    
            # Start the task executor running
            set_execution_status(True)


            wait_duration = rospy.Duration()
            for n in range(test_tasks):
                wait_duration += action_sleep

            executor.wait_for_completion(wait_duration + wait_duration)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestEntry, sys.argv)

