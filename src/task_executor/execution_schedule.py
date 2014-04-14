from strands_executive_msgs.msg import Task
from threading import Condition
from copy import deepcopy
from Queue import Queue, Empty
from collections import deque

class ExecutionSchedule(object):

    def __init__(self):
    	self.current_task = None
    	self.exection_change = Condition()
    	self.exection_queue = deque()
    	self.tasks = []

    def add_new_tasks(self, tasks):
    	""" Add new tasks to be scheduled. """
    	self.tasks.extend(tasks)

    def get_schedulable_tasks(self):
    	""" Get the tasks which are available to be scheduled. Does not include the task being executed. """
    	return deepcopy(self.tasks)

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
    		if not any(t.task_id == scheduled.task for t in self.tasks):
    			rospy.loginfo('Trying to scheduled a missed task')
    			return False

    	# now clear out the execution queue so that the new schedule comes into effect after current execution completes
    	self.exection_queue.clear()
    	self.exection_queue.extend(scheduled_tasks)
    	if self.current_task == None:
    		# start execution

	# # wait for next task
	# # blocks until its time to start execution of the next task
	# # this will be either the next task in the schedule or when task execution needs to be alter due to a change
	# # returns and enum indicating which of these it its

	# def get_current_task(self):
	# 	""" Get the next task for execution. """
	# 	return self.current_task


	# # execution complete
	# # option to reduce gaps in schedule without calling scheduler
	
	# def wait_for_execution_change(self):
	# 	""" Blocks until there is a change in execution status """
	# 	self.exection_change.acquire()
	# 	self.exection_change.wait()
	# 	self.exection_change.release()
	# 	return ExecutionStatus.NEXT_TASK