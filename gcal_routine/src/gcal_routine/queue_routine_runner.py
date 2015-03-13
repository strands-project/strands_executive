import rospy
from threading import Thread
from datetime import timedelta


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

        Thread(target=self.routine_run).start()

    def add_task(self, id, task):
        rospy.loginfo('new task %s', id)
        self.waiting_tasks[id] = task

        self.schedule_tasks()

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

    def schedule_tasks(self):
        now = self.now()
        tasks = []
        print self.waiting_tasks
        for (i, task) in self.waiting_tasks.copy().items():
            if task.end_before.secs - now.secs < 0:
                rospy.loginfo('removed outdated task %s', i)
                self.remove_task(i, task)
                continue
            if task.start_after - self.pre_schedule_delay < now:
                rospy.loginfo('task %s should be scheduled now.', i)
                self.remove_task(i, task)
                tasks.append(task)
            else:
                rospy.logdebug('task %s remains waiting.', i)
        self._schedule_tasks(tasks)

    def _schedule_tasks(self, tasks):
        rospy.loginfo('Sending %d tasks to the scheduler' % (len(tasks)))
        if len(tasks) > 0:
            if self.tasks_allowed() and self.add_tasks_srv is not None:
                self.add_tasks_srv(tasks)
            else:
                rospy.loginfo('Provided function prevented tasks'
                              ' being send to the scheduler')

    def routine_run(self):
        while not rospy.is_shutdown():
            self.schedule_tasks()
            # sleep until next check
            target = rospy.get_rostime()
            target.secs = target.secs + self.update_wait
            while self.now() < target and not rospy.is_shutdown():
                rospy.sleep(1)
