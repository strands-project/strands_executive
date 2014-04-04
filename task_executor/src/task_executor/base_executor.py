#!/usr/bin/env python

import rospy
from strands_executive_msgs.msg import Task
from strands_executive_msgs.srv import AddTask, SetExecutionStatus, GetExecutionStatus
import ros_datacentre.util as dc_util
import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion
from ros_datacentre.message_store import MessageStoreProxy
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal

class AbstractTaskExecutor(object):

    # These can be implemented by sub classes to provide hooks into the execution system

    def add_task(self, task):
        """ Called with a new task for the executor """
        self.tasks.put(task)

    def start_execution(self):
        """ Called when overall execution should  (re)start """
        pass

    def pause_execution(self):
        """ Called when overall execution should pause """
        pass

    def task_complete(self, task):
        """ Called when the given task has completed execution """
        pass



    def __init__(self):
        self.task_counter = 1
        self.msg_store = MessageStoreProxy() 
        self.executing = False
        self.active_task = None
        self.active_task_id = Task.NO_TASK
        self.nav_client = actionlib.SimpleActionClient('topological_navigation', GotoNodeAction)

        # advertise ros services
        for attr in dir(self):
            if attr.endswith("_ros_srv"):
                service=getattr(self, attr)                
                rospy.Service("/task_executor/" + attr[:-8], service.type, service)


    def get_task_types(self, action_name):
        """ 
        Returns the type string related to the action string provided.
        """
        rospy.logdebug("task action provided: %s", action_name)
        topics = rospy.get_published_topics(action_name)
        for [topic, type] in topics:            
            if topic.endswith('feedback'):
                return (type[:-8], type[:-14] + 'Goal')
        raise RuntimeError('No action associated with topic: %s'% action_name)


    def execute_task(self, task):
        self.active_task = task
        self.active_task_id = task.task_id               
        if self.active_task.node_id != '':                    
            self.start_task_navigation()
        elif self.active_task.action != '':                    
            self.start_task_action()
        else:
            rospy.logwarn('Provided task had no node_id or action %s' % self.active_task)
            self.active_task = None
            self.active_task_id = Task.NO_TASK
            

    def start_task_action(self):
        (action_string, goal_string) = self.get_task_types(self.active_task.action)
        action_clz = dc_util.load_class(dc_util.type_to_class_string(action_string))
        goal_clz = dc_util.load_class(dc_util.type_to_class_string(goal_string))

        client = actionlib.SimpleActionClient(self.active_task.action, action_clz)
        client.wait_for_server()

        argument_list = self.get_arguments(self.active_task.arguments)

        goal = goal_clz(*argument_list) 

        client.send_goal(goal, self.task_execution_complete_cb)
        
    def start_task_navigation(self):
        nav_goal = GotoNodeGoal(target = self.active_task.node_id)
        print "navigating to %s" % nav_goal
        self.nav_client.send_goal(nav_goal, self.navigation_complete_cb)

    def navigation_complete_cb(self, goal_status, result):
        # todo: check goal status to see if we really go there
        # now do the action
        self.start_task_action()


    def task_execution_complete_cb(self, goal_status, result):
        self.task_complete(self.active_task)
        self.active_task = None
        self.active_task_id = Task.NO_TASK


    def add_task_ros_srv(self, req):
        """
        Adds a task into the task execution framework.
        """
        req.task.task_id = self.task_counter
        self.task_counter += 1

        self.add_task(req.task)        
        
        return req.task.task_id
    add_task_ros_srv.type=AddTask

    def get_execution_status_ros_srv(self, req):
        return self.executing
    get_execution_status_ros_srv.type = GetExecutionStatus

    def set_execution_status_ros_srv(self, req):
        if self.executing and not req.status:
            rospy.logdebug("Pausing execution")
            self.pause_execution()
        elif not self.executing and req.status:
            rospy.logdebug("Starting execution")
            self.start_execution()
        previous = self.executing
        self.executing = req.status
        return previous
    set_execution_status_ros_srv.type = SetExecutionStatus


    def instantiate_from_string_pair(self, string_pair):
        if len(string_pair.first) == 0:
            return string_pair.second
        else:
            msg = self.msg_store.query_id(string_pair.second, string_pair.first)[0]
            print msg
            if msg == None:
                raise RuntimeError("No matching object for id %s of type %s" % (string_pair.second, string_pair.first))
            return msg

    def get_arguments(self, argument_list):
        return map(self.instantiate_from_string_pair, argument_list)



