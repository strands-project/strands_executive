import rospy
from datetime import datetime, timedelta, date
from dateutil.tz import tzlocal, tzutc
from copy import copy

from threading import Thread

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
	


def time_greater_than(t1, t2):
    """ Seems to be a bug in datetime when comparing localtz dates, so do this instead. """
    if t1.hour > t2.hour:
        return True
    elif t1.hour == t2.hour:
        if t1.minute > t2.minute:
            return True 
        elif t1.minute == t2.minute:
            if t1.second > t2.second:
                return True
            elif t1.second == t2.second:
                return t1.microsecond > t2.microsecond

    return False

def time_less_than(t1, t2):
    """ Seems to be a bug in datetime when comparing localtz dates, so do this instead. """
    if t1.hour < t2.hour:
        return True
    elif t1.hour == t2.hour:
        if t1.minute < t2.minute:
            return True 
        elif t1.minute == t2.minute:
            if t1.second < t2.second:
                return True
            elif t1.second == t2.second:
                return t1.microsecond < t2.microsecond

    return False

def delta_between(start, end):
    """
        Args:
            start (datetime.time): start time of window
            end (datetime.time): end time of window
        Returns
            datetime.timedelta: The delta of the window between start and end
    """
    sdt = datetime.combine(date.today(), start)
    if time_less_than(start, end):
        enddt = datetime.combine(date.today(), end)
    else: 
        enddt = datetime.combine(date.today(), end) + timedelta(days=1)
    return enddt - sdt




class DailyRoutine(object):
    """ An object for setting up the daily schedule of a robot. 
        Args:
            daily_start (datetime.time): The time of day when all tasks can start, local time.
            daily_end (datetime.time): The time of day when all tasks should end start, local time.
    """
    def __init__(self, daily_start, daily_end):
        super(DailyRoutine, self).__init__()

        if daily_start.tzinfo is None:
            raise RoutineException('Start time must have timezone set')

        if daily_end.tzinfo is None:
            raise RoutineException('End times must have timezone set')

        self.daily_start = daily_start
        self.routine_duration = delta_between(daily_start, daily_end)
        self.routine_tasks = []
    
    def repeat_every_day(self, tasks, times=1):
        """
        Repeat the given tasks a number of times during the day.
        """
        self.repeat_every(tasks, self.daily_start, self.routine_duration, times)



    def repeat_every_delta(self, tasks, delta=timedelta(hours=1), times=1, start_time=None, duration=None):

        if start_time is None:
            start_time = self.daily_start
        if duration is None:
            duration = self.routine_duration

        # this window is moved forward throughout the repeat
        window_start = datetime.combine(date.today(), start_time)
        window_end =  window_start + delta 
        # the past which we don't move the window
        routine_end = window_start + duration

        while window_end <= routine_end:

            if window_start == window_end:
                return
           
            print '%s.%s - %s.%s' % (window_start.hour, window_start.minute, window_end.hour, window_end.minute)
            self.repeat_every(tasks, window_start.timetz(), delta, times)

            window_start = window_end
            window_end =  window_start + delta
              

    def repeat_every_mins(self, tasks, mins=30, times=1):
        """
        Repeat the given tasks x times every n minutes
        """

        self.repeat_every_delta(tasks, timedelta(minutes=mins), times)


    def repeat_every_hour(self, tasks, hours=1, times=1):
        """
        Repeat the given tasks x times every n hours
        """
        self.repeat_every_delta(tasks, timedelta(hours=hours), times)


    def repeat_every(self, tasks, daily_start, daily_duration, times=1):
        """
        Repeat the given task a number of times during the given time window.
          Args:
            daily_start (datetime.time): The time of day when the given tasks can start, local time.
            daily_duration (datetime.timedelta): The duration of time during which the tasks can be executed
            times: the number of times to execute the tasks in the window
        """

        # make tasks a list if its not already 
        if not isinstance(tasks, list):            
            tasks = [tasks]

        if daily_start < self.daily_start:
            raise Exception('Provided daily start %s is less than overall daily start %s' % (daily_start, self.daily_start))


        daily_end = datetime.combine(date.today(), daily_start) + daily_duration
        overall_end = datetime.combine(date.today(), self.daily_start) + self.routine_duration
  

        if daily_end > overall_end:
            raise Exception('Provided daily end %s is greater than overall daily end %s for tasks %s' % (daily_end, overall_end, tasks))



        self.routine_tasks += [(tasks, (daily_start, daily_duration))] * times

    def get_routine_tasks(self):
        return self.routine_tasks




class DailyRoutineRunner(object):
    """ An object for running the daily schedule of a robot. 
        Args:
            daily_start (datetime.time): The time of day when all tasks can start, local time.
            daily_end (datetime.time): The time of day when all tasks should end start, local time.
            pre_start_window (datetime.timedelta): The duration before a task's start that it should be passed to the scheduler. Defaults to 1 hour.
    """
    def __init__(self, daily_start, daily_end, add_tasks_srv, pre_start_window=timedelta(hours=0.25), day_start_cb=None, day_end_cb=None, tasks_allowed_fn=None):
        super(DailyRoutineRunner, self).__init__()
       

        if daily_start.tzinfo is None:
            raise RoutineException('Start time must have timezone set')

        if daily_end.tzinfo is None:
            raise RoutineException('End times must have timezone set')
        
        self.daily_start = daily_start
        self.routine_duration = delta_between(daily_start, daily_end)
       
        # work out when this current run of the routine should've started/ended
        rostime_now = rospy.get_rostime()
        now = datetime.fromtimestamp(rostime_now.to_sec(), tzlocal()).time()

        self.current_routine_start = datetime.combine(date.today(), self.daily_start)       

        if time_less_than(daily_start, daily_end):
            # if we're past the day end then the current routine starts tomorrow
            if time_greater_than(now, daily_end):
                self.current_routine_start += timedelta(days=1)
        else:
            if time_less_than(now, daily_end):
                self.current_routine_start -= timedelta(days=1)

        self.current_routine_end = self.current_routine_start + self.routine_duration

        rospy.loginfo('Current day starts at %s' % self.current_routine_start)
        rospy.loginfo('Current day ends at %s' % self.current_routine_end)

        if self.current_routine_start.tzinfo is None:
            raise RoutineException()

        if self.current_routine_end.tzinfo is None:
            raise RoutineException()

        # the tasks which need to be performed every day, tuples of form (daily_start, daily_end, task)
        self.routine_tasks = []
        self.add_tasks_srv = add_tasks_srv

        if tasks_allowed_fn is None:
            self.tasks_allowed = self._tasks_allowed_fn
        else:
            self.tasks_allowed = tasks_allowed_fn

        self.days_off = []
        self.dates_off = []

        # check for local spellings!
        self.all_days = []
        day_today = datetime.fromtimestamp(rospy.get_rostime().to_sec(), tz=tzlocal())
        for n in range(0,7):
            self.all_days.append(day_today.strftime("%A"))
            day_today += timedelta(days=1)

        self.pre_schedule_delay = rospy.Duration(pre_start_window.total_seconds())

        self.day_start_cb = day_start_cb
        self.day_end_cb = day_end_cb
        Thread(target=self._start_and_end_day).start()

    def add_day_off(self, day_name):
        """ Add a day of the week, e.g. Saturday, on which the routing should not be run """
        if day_name not in self.all_days:
            raise Exception('Day name %s not allowed. Must be one of %s' % (day_name, self.all_days))
        self.days_off.append(day_name)

    def add_date_off(self, date):
        """ Add a datetime.date on which the robot should not work. """
        self.dates_off.append(date)

    def day_off(self):
        datetime_today = datetime.fromtimestamp(rospy.get_rostime().to_sec(), tz=tzlocal())
        day_today = datetime_today.strftime("%A")
        date_today = datetime_today.date()

        return (day_today in self.days_off) or (date_today in self.dates_off)

    def _tasks_allowed_fn(self):        
        return True

    def _start_and_end_day(self):
        """ Runs a loop which triggers the start and end of day callbacks """

        loop_delay = rospy.Duration(1)

        while not rospy.is_shutdown():
 
            # again, using sleeps rather than timers due to slightly odd sim time behaviour of the latter
            now = datetime.fromtimestamp(rospy.get_rostime().to_sec(), tz=tzlocal())

            # wait for a the pre-delay before the start of the day to instantiate the day's tasks
            pre_delayed_start = self.current_routine_start - timedelta(seconds=self.pre_schedule_delay.to_sec())
            while now < pre_delayed_start and not rospy.is_shutdown():
                rospy.sleep(loop_delay)
                now = datetime.fromtimestamp(rospy.get_rostime().to_sec(), tz=tzlocal())

            self._new_day()

            # wait for the start of the day
            while now < self.current_routine_start and not rospy.is_shutdown():
                rospy.sleep(loop_delay)
                now = datetime.fromtimestamp(rospy.get_rostime().to_sec(), tz=tzlocal())

            if self.day_start_cb is not None and not rospy.is_shutdown():
                rospy.loginfo('triggering day start cb at %s' % now)
                self.day_start_cb()

            while now < self.current_routine_end and not rospy.is_shutdown():
                rospy.sleep(loop_delay)
                now = datetime.fromtimestamp(rospy.get_rostime().to_sec(), tz=tzlocal())

            if self.day_end_cb is not None and not rospy.is_shutdown():
                rospy.loginfo('triggering day end cb at %s' % now)
                self.day_end_cb()

            # update routine window
            self.current_routine_start += timedelta(days=1)
            self.current_routine_end += timedelta(days=1)


    """
        Adds a task to the daily routine.
        Args:
            routines: A list of tuples where each tuple is (task_list, window) where task_list is a list of task and window is a tuple (start, duration) representing a time window in which to execute those tasks each day.
    """
    def add_tasks(self, routines):

        # add tasks to daily route
        new_routine = []

        for task_tuple in routines:
            

            daily_start = task_tuple[1][0]            
            daily_duration = task_tuple[1][1]

            for task in task_tuple[0]:
        
                # else, add to daily routine
                routine = (daily_start, daily_duration, task)
                self.routine_tasks.append(routine)
                new_routine.append(routine)

        # see if we can still schedule any of these today
        todays_tasks = self._instantiate_tasks_for_today(new_routine)
        # the false means an exception is not thrown for any out of window tasks
        self._create_routine(todays_tasks, False)

        
    def _new_day(self):
        """
            Should be called each new day, in advance of the overall daily start time.
        """

        todays_tasks = self._instantiate_tasks_for_today(self.routine_tasks)
        self._create_routine(todays_tasks)


    def _instantiate_tasks_for_today(self, routine_tasks):
        # instantiate the daily tasks with release windows for today
        todays_tasks = [self._instantiate_for_day(self.current_routine_start, *task_tuple) for task_tuple in routine_tasks]
        return todays_tasks

    def _create_routine(self, tasks, throw=True):

        schedule_now, schedule_later = self._queue_for_scheduling(tasks, throw)

        rospy.loginfo('Scheduling %s tasks now and %s later' % (len(schedule_now), len(schedule_later)))

        self._delay_tasks(schedule_later)
        self._schedule_tasks(schedule_now)


    def _queue_for_scheduling(self, tasks, throw=True):
        now = rospy.get_rostime()

        schedule_now = []
        schedule_later = []

        for task in tasks:

            # print task.max_duration.secs
            # print task.start_after.secs
            # print task.end_before.secs
            # print (now).secs
            # print (now + task.max_duration).secs


            # check we're not too late
            if now + task.max_duration > task.end_before:
                # 
                if throw:
                    raise RoutineException('%s is too late to schedule task %s' % (now.secs, task))
                else:
                    rospy.logdebug('Ignoring task for today')
            else:
                # if we're in the the window when this should be scheduled
                if now > (task.start_after - self.pre_schedule_delay):
                    schedule_now.append(task)
                else:
                    schedule_later.append(task)

        return schedule_now, schedule_later


    def _delay_tasks(self, tasks):
        """
            Delays call to the scheduer until the first of these needs to start, then reruns check for scheduling
        """

        if len(tasks) == 0:
            return

        # find the soonest start date of all tasks

        # an arbitrary large date
        min_start_date = rospy.Time(unix_time(self.current_routine_end))
        for task in tasks:
            if task.start_after < min_start_date:       
                # print task.max_duration.secs
                # print task.start_after.secs
                # print task.end_before.secs
                # print (now).secs
                # print (now + task.max_duration).secs    
                min_start_date = task.start_after 

        # print 'min start at %s' % min_start_date.secs

        # only delay up to the pre-scheduler window 
        min_start_date = min_start_date - self.pre_schedule_delay

        now = rospy.get_rostime()


        # print 'min start at %s' % min_start_date.secs
        # print '      now at %s' % now.secs

        delay = min_start_date - now

        assert delay.secs > 0


        # print 'delaying %s' % delay.secs
        self._delay_scheduling(tasks, delay)


    # separated out to try differeing approaches
    def _delay_scheduling(self, tasks, delay):
        rospy.loginfo('Delaying %s tasks for %s secs' % (len(tasks), delay.secs))

        def _check_tasks():
            # using a sleep instead of a timer as the timer seems flakey with sim time            
            target = rospy.get_rostime() + delay
            # make sure we can be killed here
            while rospy.get_rostime() < target and not rospy.is_shutdown():
                rospy.sleep(1)
            # handle being shutdown
            if not rospy.is_shutdown():
                self._create_routine(tasks)

        Thread(target=_check_tasks).start()


    def _schedule_tasks(self, tasks):
        rospy.loginfo('Sending %s tasks to the scheduler' % (len(tasks)))
        if len(tasks) > 0:
            if not self.day_off() and self.tasks_allowed():
                self.add_tasks_srv(tasks)
            else:
                rospy.loginfo('Provided function prevented tasks being send to the scheduler')

    def _instantiate_for_day(self, start_of_day, daily_start, daily_duration, task):
        """ 
            Create a copy of the given task with start and end times instantiated from input variables relative to the start date provided.
        """
        instantiated_task = copy(task)
        release_date = datetime.combine(start_of_day.date(), daily_start)
        end_date = release_date + daily_duration
        instantiated_task.start_after = rospy.Time(unix_time(release_date))
        instantiated_task.end_before = rospy.Time(unix_time(end_date))
        # print '%s (%s)' % (release_date, instantiated_task.start_after.secs)
        # print '%s (%s)' % (end_date, instantiated_task.end_before.secs)

        return instantiated_task






