from strands_executive_msgs.msg import Task
from threading import Event
from copy import deepcopy
from Queue import Queue, Empty
from collections import deque

import rospy

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

    def get_execution_queue(self):
        """ Get the tasks which are queued for execution. """
        return deepcopy(self.execution_queue)


    def execute_next_task(self):
        """
        Sets the head of the execution queue to the current task, removes this from schedulable tasks, and notifies the wait_for_execution_change method. 
        """
        if len(self.execution_queue) > 0:
            self.current_task = self.execution_queue.popleft()
            # remove current task from schedulable tasks
            print len(self.tasks)
            self.tasks = [t for t in self.tasks if t.task_id != self.current_task.task_id]            
            print len(self.tasks)
            self.execution_change.set()
        else:
            self.current_task = None

    def execution_delay_cb(self, event):
        rospy.logdebug('timer for execution delay fired')
        self.next_in_schedule()

    def next_in_schedule(self):
        """
        Checks whether the next action can be executed now (i.e. the current time is within its constraints). If not, delays execution suitably.
        """
        if len(self.execution_queue) > 0:
            now = rospy.get_rostime()
            next_task = self.execution_queue[0]

            rospy.loginfo('start window: %s.%s' % (next_task.start_after.secs, next_task.start_after.nsecs))
            rospy.loginfo('         now: %s.%s' % (now.secs, now.nsecs))
            
            # if the start window is open
            if next_task.start_after <= now:
                rospy.loginfo('start window is open')
                self.execute_next_task()
            else:     
                exe_delay = next_task.start_after - now
                rospy.loginfo('need to delay %s.%s for execution' % (exe_delay.secs, exe_delay.nsecs))
                self.execution_delay_timer = rospy.Timer(exe_delay, self.execution_delay_cb, oneshot=True)
        else:
            self.current_task = None


    def task_complete(self, task):
        assert task != None
        assert self.current_task != None
        assert task.task_id == self.current_task.task_id
        # no time info yet
        self.next_in_schedule()


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
            self.next_in_schedule()            



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


