#!/usr/bin/env python

import rospy
import Queue
from strands_executive_msgs.msg import Task
from strands_executive_msgs.srv import AddTask



class TaskExecutor(object):
    def __init__(self):
        rospy.init_node("task_executor")
        self.task_counter = 0
        self.tasks = Queue.Queue()


        # advertise ros services
        for attr in dir(self):
            if attr.endswith("_ros_srv"):
                service=getattr(self, attr)
                rospy.Service(rospy.get_name() + "/" + attr[:-8], service.type, service)


    def add_task_ros_srv(self, req):
        """
        Adds a task into the task execution framework.
        """
        req.task = self.task_counter
        self.task_counter += 1
        self.tasks.put(req.task)
        return req.task
    add_task_ros_srv.type=AddTask




if __name__ == '__main__':
    executor = TaskExecutor()    
    rospy.spin()
