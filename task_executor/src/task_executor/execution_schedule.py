from strands_executive_msgs.msg import Task
from threading import Event
from copy import deepcopy
from Queue import Queue, Empty
from collections import deque
from task_executor.utils import rostime_to_python, rosduration_to_python


import rospy

# def enum(*sequential, **named):
#     """ 
#     Pre-v3 emum support from http://stackoverflow.com/questions/36932/how-can-i-represent-an-enum-in-python
#     """
#     enums = dict(zip(sequential, range(len(sequential))), **named)
#     return type('Enum', (), enums)

# ExecutionStatus = enum('ZERO', 'ONE', 'TWO')

class ExecutionSchedule(object):

    def __init__(self, travel_duration_fn):
        """
        travel_duration_fn: a function which takes a string and returns the expected travel duration to it as a rospy.Duration assuming the travel will happen immediately
        """
    	self.current_task = None
    	self.execution_change = Event()
    	self.execution_queue = deque()
    	self.tasks = []        
        self.execution_delay_timer = None
        self._travel_duration_fn = travel_duration_fn

    def add_new_tasks(self, tasks):
    	""" Add new tasks to be scheduled. """
    	self.tasks.extend(tasks)

    def get_schedulable_tasks(self):
    	""" Get the tasks which are available to be scheduled. Does not include the task being executed. """
    	return deepcopy(self.tasks)

    def get_execution_queue(self):
        """ Get the tasks which are queued for execution. """
        return deepcopy(self.execution_queue)

    def get_execution_queue_length(self):
        """ Get the number of tasks which are queued for execution. """
        return len(self.execution_queue)


    def clear_schedule(self):
        """ Removes all tasks from the execution queue and task list """
        self.execution_queue.clear()        
        del self.tasks[:]

    def remove_task_with_id(self, task_id):
        # remove from list of available tasks
        prior_tasks_len = len(self.tasks)
        prior_queue_len = len(self.execution_queue)
        self.tasks = [t for t in self.tasks if t.task_id != task_id]            
        # also remove from execution queue, order and contents can be different from self.tasks
        self.execution_queue = deque([t for t in self.execution_queue if t.task_id != task_id])            
        # if one of these got shorter then we did good
        return len(self.tasks)  < prior_tasks_len or len(self.execution_queue) < prior_queue_len

    def remove_tasks(self, tasks):
        for task in tasks:
            # massively inefficient!
            self.remove_task_with_id(task.task_id)

    def execute_next_task(self):
        """
        Pops the head of the execution queue to the current task, removes this from schedulable tasks, and notifies the wait_for_execution_change method. 
        """
        if len(self.execution_queue) > 0:
            self.current_task = self.execution_queue.popleft()
            # remove current task from schedulable tasks
            self.tasks = [t for t in self.tasks if t.task_id != self.current_task.task_id]            
            self.execution_change.set()
        else:
            self.current_task = None

    def execution_delay_cb(self, event):
        rospy.logdebug('timer for execution delay fired')
        self.execution_delay_timer = None
        self.next_in_schedule()

    def next_in_schedule(self):
        """
        Checks whether the next action can be executed now (i.e. the current time is within its constraints). If not, delays execution suitably.
        """
        if len(self.execution_queue) > 0:
            now = rospy.get_rostime()
            next_task = self.execution_queue[0]
            # get travel time to next task. 
            travel_time_to_next_task = self._travel_duration_fn(next_task.start_node_id)

            rospy.loginfo('start window: %s' % rostime_to_python(next_task.start_after).time())
            rospy.loginfo('         now: %s' % rostime_to_python(now).time())
            rospy.loginfo(' travel time:  %s' % rosduration_to_python(travel_time_to_next_task))
            
            # the task can be executed if we can travel to the task in time for the
            # the start window to open
            if now >= (next_task.start_after - travel_time_to_next_task):
                rospy.loginfo('start window is open')
                self.execute_next_task()
            else:     
                # delay is the difference between now and the time to start navigating
                exe_delay = (next_task.start_after - travel_time_to_next_task) - now
                rospy.loginfo('need to delay %s.%s for execution' % (exe_delay.secs, exe_delay.nsecs))
                self.current_task = None
                self.execution_delay_timer = rospy.Timer(exe_delay, self.execution_delay_cb, oneshot=True)
        else:
            self.current_task = None


    def task_complete(self, task):
        assert task is not None
        assert self.current_task is not None
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
    		rospy.logwarn('Number of scheduled tasks mismatch: given %s, had %s' % (len(scheduled_tasks),len(self.tasks)))
    		return False

    	for scheduled in scheduled_tasks:
    		if not any(t.task_id == scheduled.task_id for t in self.tasks):
    			rospy.logwarn('Trying to scheduled a missed task')
    			return False

    	# now clear out the execution queue so that the new schedule comes into effect after current execution completes
    	self.execution_queue.clear()
    	self.execution_queue.extend(scheduled_tasks)
    	
        # if nothing is executing, make sure something starts
        if self.current_task is None:
            # nothing may be executing becuase we're waiting for delayed execution
            if self.execution_delay_timer is not None:
                self.execution_delay_timer.shutdown()
            self.next_in_schedule()
        # else:
        #     rospy.logwarn('current task is still: %s', self.current_task)



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


