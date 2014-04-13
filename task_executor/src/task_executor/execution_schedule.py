from strands_executive_msgs.msg import Task
from threading import Condition


class ExecutionSchedule(object):

    def __init__(self):
    	self.current_task = None
    	self.exection_change = Condition()
    	self.tasks = []

    def add_new_tasks(self, tasks):
    	""" Add new tasks to be scheduled. """
    	self.tasks.extend(tasks)

    def get_schedulable_tasks(self):
    	""" Get the tasks which are available to be scheduled. Does not include the task being executed. """
    	return self.tasks

    def set_schedule(self, order, start_times):
    	pass

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