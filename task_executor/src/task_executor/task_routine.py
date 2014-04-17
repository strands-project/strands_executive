import rospy
from datetime import *
from copy import copy
from dateutil.tz import *

_epoch = datetime.utcfromtimestamp(0).replace(tzinfo=tzutc())

class RoutineException(Exception): 
    pass

def unix_time(dt):
    """
    Converts a datetime object to seconds since epoch
    """    
    delta = dt - _epoch
    return delta.total_seconds()

def time_to_secs(time):
    """ 
    Coverts a datetime.time object to seconds.
    """
    return (time.hour * 60 * 60) + (time.minute * 60) + (time.second) + (time.microsecond/1000.0)
	
    # start by providing bounds on daily exectuion

    # given a task or list of tasks (which could be produce by that task at all waypoints)

    # do it n times in a specific window,  - scheduled for proposal x secs before the window

    # do it n times every X duration 

def start_of_the_day(day_delta=0): 
    """
        Get the datetime at the start of the day requested, i.e. the last midnight which has passed. All done in UTC

        Args:
            day_delta (int, optional): The relative day to today. 0 is today, 1 is tomorrow, -1 is yesterday. Defaults to 0.
        Returns:
            datetime.datetime: The date time at the start of the day 
    """
    return datetime.fromordinal(datetime.utcfromtimestamp(rospy.get_rostime().to_sec()).toordinal())

class DailyRoutine(object):
    """ An object for setting up the daily schedule of a robot. 
        Args:
            daily_start (datetime.time): The time of day when all tasks can start, local time.
            daily_end (datetime.time): The time of day by when all tasks should end, local time.
            pre_start_window (datetime.timedelta): The duration before a task's start that it should be passed to the scheduler. Defaults to 1 hour.
    """
    def __init__(self, daily_start, daily_end, pre_start_window=timedelta(hours=1)):
        super(DailyRoutine, self).__init__()
        self.daily_start = daily_start
        self.daily_end = daily_end
        # the tasks which need to be performed every day, tuples of form (daily_start, daily_end, task)
        self.routine_tasks = []
        self.pre_schedule_delay = rospy.Duration(pre_start_window.total_seconds())

    """
        Does some basic checking on the task.
        Args:
            daily_start (datetime.time): The time of day when the task can start.
            daily_end (datetime.time): The time of day by when the task should end.
    """
    def sanity_check_task(self, daily_start, daily_end, task):
        end_secs = time_to_secs(daily_end)
        start_secs = time_to_secs(daily_start)
        if end_secs <= start_secs:
            raise RoutineException('Task window is negative: %s to %s' % (daily_start, daily_end))
        if end_secs - start_secs < task.expected_duration.secs:
            raise RoutineException('Task window is not large enough for task: %s to %s for task of duration %s' % (daily_start, daily_end, timedelta(seconds=task.expected_duration)))


    """
        Adds a task to the daily routine.
        Args:
            daily_start (datetime.time): The time of day when the task can start, local time.
            daily_end (datetime.time): The time of day by when the task should end, local time.
    """
    def add_task(self, task, daily_start, daily_end):

        # convert to utc

        # bound by daily activity window

        self.sanity_check_task(daily_start, daily_end, task)

        # if running, this needs to be added to the active routine


        # else, add to daily routine
        self.routine_tasks.append((daily_start, daily_end, task))
        
    """
        Should be called each new day, in advance of the overall daily start time.
    """
    def _new_day(self):

        start_of_today = start_of_the_day()
        # 
        for task_tuple in self.routine_tasks:
            task = self._instantiate_for_today(start_of_today, *task_tuple)
            self._queue_for_scheduling(task)

    def _instantiate_for_today(self, start_of_today, daily_start, daily_end, task):
        """ 
            Create a copy of the given task with start and end times instantiated from input variables relative to the start date provided.
        """
        instantiated_task = copy(task)
        release_date = datetime.combine(start_of_today.date(), daily_start)
        end_date = datetime.combine(start_of_today.date(), daily_end)
        instantiated_task.start_after = rospy.Time(unix_time(release_date))
        instantiated_task.end_before = rospy.Time(unix_time(end_date))
        print '%s (%s)' % (release_date, instantiated_task.start_after.secs)
        print '%s (%s)' % (end_date, instantiated_task.end_before.secs)

        return instantiated_task


    def _queue_for_scheduling(self, task):
        now = rospy.get_rostime()

        print task.expected_duration.secs
        print task.start_after.secs
        print task.end_before.secs
        print (now).secs
        print (now + task.expected_duration).secs

        # check we're not too late
        if now + task.expected_duration > task.end_before:
            raise RoutineException('%s is too late to schedule task %s' % (now.secs, task))

        # if we're in the the window when this should be scheduled
        if now > (task.start_after - self.pre_schedule_delay):
            print "schedule it"
        else:
            print "delay it"







