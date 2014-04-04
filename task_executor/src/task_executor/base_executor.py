#!/usr/bin/env python

import rospy
from strands_executive_msgs.msg import Task
from strands_executive_msgs.srv import AddTask
import ros_datacentre.util as dc_util
import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion
from ros_datacentre.message_store import MessageStoreProxy


class AbstractTaskExecutor(object):
    def __init__(self):
        self.task_counter = 0
        self.msg_store = MessageStoreProxy() 

        # advertise ros services

        for attr in dir(self):
            if attr.endswith("_ros_srv"):
                service=getattr(self, attr)                
                rospy.Service(rospy.get_name() + "/" + attr[:-8], service.type, service)


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



    def add_task_ros_srv(self, req):
        """
        Adds a task into the task execution framework.
        """
        req.task.task_id = self.task_counter
        self.task_counter += 1

        self.add_task(req.task)        

        (action_string, goal_string) = self.get_task_types(req.task.action)
        action_clz = dc_util.load_class(dc_util.type_to_class_string(action_string))
        goal_clz = dc_util.load_class(dc_util.type_to_class_string(goal_string))


        client = actionlib.SimpleActionClient(req.task.action, action_clz)
        client.wait_for_server()

        argument_list = self.get_arguments(req.task.arguments)

        print argument_list

        goal = goal_clz(*argument_list)        
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(5.0))

        return req.task.task_id
    add_task_ros_srv.type=AddTask

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



