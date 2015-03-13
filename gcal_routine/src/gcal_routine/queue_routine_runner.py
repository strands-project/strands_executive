import rospy
from threading import Thread
from datetime import datetime
from datetime import timedelta
from dateutil.tz import tzutc

_epoch = datetime.utcfromtimestamp(0).replace(tzinfo=tzutc())


def unix_time(dt):
    """
    Converts a datetime object to seconds since epoch
    """
    delta = dt - _epoch
    return delta.total_seconds()


class GCalRoutineRunner(object):
    """ An object for running the daily schedule of a robot.
    """

    def __init__(self, add_tasks_srv,
                 pre_start_window=timedelta(hours=0.25), tasks_allowed_fn=None):
        super(GCalRoutineRunner, self).__init__()

        self.pre_schedule_delay = rospy.Duration(pre_start_window.total_seconds())

        self.routine_tasks = []
        self.add_tasks_srv = add_tasks_srv

        if tasks_allowed_fn is None:
            self.tasks_allowed = self._tasks_allowed_fn
        else:
            self.tasks_allowed = tasks_allowed_fn

        Thread(target=self._start_and_end_day).start()

    def _tasks_allowed_fn(self):
        return True

    """
        Adds a task to the daily routine.
        Args:
            routines: A list of tuples where each tuple is (task_list, window) where task_list is a list of task and window is a tuple (start, duration) representing a time window in which to execute those tasks each day.
    """
    def add_tasks(self, todays_tasks):
        self._create_routine(todays_tasks, False)

    def _create_routine(self, tasks, throw=True):

        schedule_now, schedule_later = self._queue_for_scheduling(tasks, throw)

        rospy.loginfo('Scheduling %s tasks now and %s later' % (len(schedule_now), len(schedule_later)))

        self._delay_tasks(schedule_later)
        self._schedule_tasks(schedule_now)


    def _delay_tasks(self, tasks):
        """
            Delays call to the scheduer until the first of these needs to start, then reruns check for scheduling
        """

        if len(tasks) == 0:
            return

        # find the soonest start date of all tasks

        # an arbitrary large date
        min_start_date = rospy.Time(unix_time(datetime.now()+timedelta(year=1)))
        for task in tasks:
            if task.start_after < min_start_date:
                min_start_date = task.start_after

        # only delay up to the pre-scheduler window
        min_start_date = min_start_date - self.pre_schedule_delay

        now = rospy.get_rostime()

        # print 'min start at %s' % min_start_date.secs
        # print '      now at %s' % now.secs

        delay = min_start_date - now

        # print 'delaying %s' % delay.secs
        self._delay_scheduling(tasks, delay)

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
            if self.tasks_allowed():
                self.add_tasks_srv(tasks)
            else:
                rospy.loginfo('Provided function prevented tasks being send to the scheduler')

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
