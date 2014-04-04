#!/usr/bin/env python

import rospy
from Queue import Queue, Empty
from strands_executive_msgs.msg import Task
from task_executor.base_executor import AbstractTaskExecutor


class FIFOTaskExecutor(AbstractTaskExecutor):
    def __init__(self):
        # init node first, must be done before call to super init for service advertising to work
        rospy.init_node("task_executor")
        # init superclasses
        super( FIFOTaskExecutor, self ).__init__()
        self.tasks = Queue()


    def add_task(self, task):
        """ Called with a new task for the executor """
        self.tasks.put(task)

    def run_executor(self):
        r = rospy.Rate(1) # 1hz
        
        while not rospy.is_shutdown():

            if self.executing:
                if self.active_task_id == Task.NO_TASK:
                    print "need a task"
                    try:
                        task = self.tasks.get(False)
                        self.execute_task(task)
                    except Empty, e:
                        pass
                else:
                    print "executing task %s" % self.active_task_id
                
            r.sleep()
        

if __name__ == '__main__':
    executor = FIFOTaskExecutor()    
    executor.run_executor()    
