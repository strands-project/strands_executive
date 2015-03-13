import rospy
from threading import Thread
from datetime import timedelta
from .tools import rostime_str


class GCalRoutineRunner(object):
    """ An object for running the daily schedule of a robot.
    """

    def __init__(self, add_tasks_srv,
                 pre_start_window=timedelta(hours=0.25), update_wait=10,
                 tasks_allowed_fn=None):
        super(GCalRoutineRunner, self).__init__()

        self.pre_schedule_delay = rospy.Duration(
            pre_start_window.total_seconds())
        self.add_tasks_srv = add_tasks_srv
        self.update_wait = update_wait
        self.waiting_tasks = {}

        if tasks_allowed_fn is None:
            self.tasks_allowed = self._tasks_allowed_fn
        else:
            self.tasks_allowed = tasks_allowed_fn

        Thread(target=self._routine_run).start()

    def add_task(self, id, task):
        rospy.loginfo('adding %s to the set of tasks', id)
        self.waiting_tasks[id] = task

    def remove_task(self, i, task):
        rospy.loginfo('remove task %s', i)
        if i in self.waiting_tasks:
            self.waiting_tasks.pop(i)
        else:
            rospy.logwarn('task %s to be deleted was not in'
                          'the waiting set. It may be from the past')

    def _tasks_allowed_fn(self):
        return True

    def now(self):
        return rospy.get_rostime()

    def get_schedulable_tasks(self):
        now = self.now()
        tasks = []
        for (i, task) in self.waiting_tasks.copy().items():
            if task.end_before.secs - now.secs < 0:
                rospy.loginfo('removed outdated task %s that was'
                              ' planned to end until %s',
                              i, rostime_str(task.end_before))
                self.remove_task(i, task)
                continue
            if task.start_after - self.pre_schedule_delay < now:
                rospy.loginfo('task %s to start after %s should'
                              ' be scheduled now.',
                              i, rostime_str(task.start_after))
                self.remove_task(i, task)
                tasks.append(task)
            else:
                rospy.logdebug('task %s remains waiting.', i)
        return tasks

    def schedule_tasks(self, tasks):
        rospy.loginfo('Sending %d tasks to the scheduler' % (len(tasks)))
        if len(tasks) > 0:
            if self.tasks_allowed():
                self.add_tasks_srv(tasks)
            else:
                rospy.loginfo('Provided function prevented tasks'
                              ' being send to the scheduler')

    def _routine_run(self):
        while not rospy.is_shutdown():
            rospy.logdebug('looking for schedule-able tasks')
            t = self.get_schedulable_tasks()
            rospy.logdebug('found %d task suitable for scheduling', len(t))
            if self.add_tasks_srv is not None:
                self.schedule_tasks(t)
            # sleep until next check
            target = rospy.get_rostime()
            target.secs = target.secs + self.update_wait
            while self.now() < target and not rospy.is_shutdown():
                rospy.sleep(1)
