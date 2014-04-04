#!/usr/bin/env python

import rospy
import Queue
from strands_executive_msgs.msg import Task
from task_executor.base_executor import AbstractTaskExecutor


class FIFOTaskExecutor(AbstractTaskExecutor):
    def __init__(self):
        # init node first, must be done before call to super init for service advertising to work
        rospy.init_node("task_executor")
        # init superclasses
        super( FIFOTaskExecutor, self ).__init__()
        self.tasks = Queue.Queue()


    def add_task(self, task):
        print task
        self.tasks.put(task)


if __name__ == '__main__':
    executor = FIFOTaskExecutor()    
    rospy.spin()
