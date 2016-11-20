#!/usr/bin/env python

import rospy

from strands_executive_msgs.srv import AddTasks
from gcal_routine.queue_routine_runner import GCalRoutineRunner
from gcal_routine.tools import GCal
from datetime import timedelta

if __name__ == '__main__':

    rospy.init_node("gcal_routine", log_level=rospy.INFO)

    while not rospy.is_shutdown() and rospy.get_rostime().secs == 0:
        pass

    stand_alone_test = rospy.get_param('~stand_alone_test', False)

    add_tasks_srv_name = '/task_executor/add_tasks'
    rospy.loginfo("Waiting for task_executor service...")
    if not stand_alone_test:
        rospy.wait_for_service(add_tasks_srv_name)
    rospy.loginfo("Done")
    add_tasks_srv = rospy.ServiceProxy(add_tasks_srv_name, AddTasks)

    runner = GCalRoutineRunner(add_tasks_srv,
                               pre_start_window=timedelta(
                                   minutes=rospy.get_param(
                                       '~pre_start_window_min',
                                       30)),
                               update_wait=rospy.get_param(
                                   '~routine_update_wait_sec',
                                   10))
    if rospy.has_param('~minTimeDelta'):
        minTimeDelta = rospy.get_param('~minTimeDelta')
        rospy.loginfo('using minTimeDelta: %d', minTimeDelta)
    else:
        minTimeDelta = None
    if rospy.has_param('~maxTimeDelta'):
        maxTimeDelta = rospy.get_param('~maxTimeDelta')
    else:
        maxTimeDelta = None

    if stand_alone_test:
        gcal = GCal(rospy.get_param('~calendar',
                                    'henry.strands%40hanheide.net'),
                    rospy.get_param('~key',
                                    'AIzaSyC1rqV2yecWwV0eLgmoQH7m7PdLNX1p6a0'),
                    update_wait=rospy.get_param('~gcal_poll_wait_sec', 10),
                    add_cb=runner.add_task, remove_cb=runner.remove_task,
                    minTimeDelta=minTimeDelta, maxTimeDelta=maxTimeDelta,
                    time_critical=True
                    )
    else:
        gcal = GCal(rospy.get_param('~calendar',
                                    'henry.strands%40hanheide.net'),
                    rospy.get_param('~key',
                                    'AIzaSyC1rqV2yecWwV0eLgmoQH7m7PdLNX1p6a0'),
                    update_wait=rospy.get_param('~gcal_poll_wait_sec', 60),
                    add_cb=runner.add_task, remove_cb=runner.remove_task,
                    minTimeDelta=minTimeDelta, maxTimeDelta=maxTimeDelta,
                    time_critical=True
                    )
    added = []
    removed = []
    gcal.update(added, removed)
    if stand_alone_test:
        gcal.shift_to_now()
    gcal.start_worker()

    rospy.spin()
