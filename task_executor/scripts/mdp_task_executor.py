#!/usr/bin/env python

import rospy
from Queue import Queue, Empty
from strands_executive_msgs.msg import Task, ExecutionStatus, DurationMatrix, DurationList
from strands_executive_msgs.srv import GetSchedule
from task_executor.sm_base_executor import AbstractTaskExecutor
from task_executor.base_executor import BaseTaskExecutor
from threading import Thread
from task_executor.execution_schedule import ExecutionSchedule
from operator import attrgetter
import copy
from math import floor

class MDPTaskExecutor(BaseTaskExecutor):

    def __init__(self):
        # init node first, must be done before call to super init for service advertising to work
        rospy.init_node("task_executor", log_level=rospy.INFO)

        # init superclasses
        super( MDPTaskExecutor, self ).__init__()
        self.advertise_services()

    def _convert_task_to_mdp_action(self, task):
        """ Converts a Task to a MdpAction """
        print task        

    def add_tasks(self, tasks):
        """ Called with new tasks for the executor """
        mdp_tasks = [self._convert_task_to_mdp_action(task) for task in tasks]


    def start_execution(self):
        """ Called when overall execution should  (re)start """
        pass

    def pause_execution(self):
        """ Called when overall execution should pause """
        pass

    def task_complete(self, task):
        """ Called when the given task has completed execution """
        pass

    def task_succeeded(self, task):
        """ Called when the given task has completed execution successfully """
        self.task_complete(task)

    def task_failed(self, task):
        """ Called when the given task has completed execution but failed """
        self.task_complete(task)


    def task_demanded(self, demanded_task, currently_active_task):
        """ Called when a task is demanded. self.active_task is the demanded task (and is being executed) and previously_active_task was the task that was being executed (which could be None) """
        pass

    def cancel_task(self, task_id):
        """ Called when a request is received to cancel a task. The currently executing one is checked elsewhere. """
        return False

    def clear_schedule(self):
        """ Called to clear all tasks from schedule, with the exception of the currently executing one. """
        pass


if __name__ == '__main__':
    executor = MDPTaskExecutor()        
    rospy.spin()


# create a schedule class which handles blocking until execution and manages the various changes

