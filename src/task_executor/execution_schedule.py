from strands_executive_msgs.msg import Task
from threading import Event
from copy import deepcopy
from Queue import Queue, Empty
from collections import deque


# def enum(*sequential, **named):
#     """ 
#     Pre-v3 emum support from http://stackoverflow.com/questions/36932/how-can-i-represent-an-enum-in-python
#     """
#     enums = dict(zip(sequential, range(len(sequential))), **named)
#     return type('Enum', (), enums)

# ExecutionStatus = enum('ZERO', 'ONE', 'TWO')

class ExecutionSchedule(object):

    def __init__(self):
    	self.current_task = None
    	self.execution_change = Event()
    	self.execution_queue = deque()
    	self.tasks = []        

    def add_new_tasks(self, tasks):
    	""" Add new tasks to be scheduled. """
    	self.tasks.extend(tasks)

    def get_schedulable_tasks(self):
    	""" Get the tasks which are available to be scheduled. Does not include the task being executed. """
    	return deepcopy(self.tasks)

    def execute_next_task(self):
        self.current_task = self.execution_queue.popleft()
        self.execution_change.set()


    def set_schedule(self, scheduled_tasks):
    	"""
    	Receive a list of tasks in order of execution, with their execution times set.
    	Returns true if the schedule was 
    	"""

    	# check that the tasks that were scheduled are the ones we're waiting on

    	if len(scheduled_tasks) != len(self.tasks):
    		rospy.loginfo('Number of scheduled tasks mismatch')
    		return False

    	for scheduled in scheduled_tasks:
    		if not any(t.task_id == scheduled.task_id for t in self.tasks):
    			rospy.loginfo('Trying to scheduled a missed task')
    			return False

    	# now clear out the execution queue so that the new schedule comes into effect after current execution completes
    	self.execution_queue.clear()
    	self.execution_queue.extend(scheduled_tasks)
    	
        # if nothing is executing, make sure something starts
        if self.current_task == None:
            self.execute_next_task()            



    def get_current_task(self):
        """ Get the next task for execution. """
        return self.current_task


    def wait_for_execution_change(self, timeout):
        """ 
        Blocks until current_task changes value 
        """
        self.execution_change.wait(timeout)

        # if change was notified
        if self.execution_change.is_set():
            # reset to cause future calls to block until next notification
            self.execution_change.clear()
            return True
        else:
            # timeout
            return False


