#!/usr/bin/env python

import rospy

from strands_executive_msgs.srv import AddTasks
from gcal_routine.queue_routine_runner import GCalRoutineRunner
from gcal_routine.tools import GCal

if __name__ == '__main__':

    rospy.init_node("gcal_routine", log_level=rospy.INFO)

    # wait for simulated time to kick in as rospy.get_rostime() is 0 until first clock message received
    while not rospy.is_shutdown() and rospy.get_rostime().secs == 0:
        pass

    stand_alone_test = rospy.get_param('~stand_alone_test', False)

    add_tasks_srv_name = '/task_executor/add_tasks'
    rospy.loginfo("Waiting for task_executor service...")
    if not stand_alone_test:
        rospy.wait_for_service(add_tasks_srv_name)
    rospy.loginfo("Done")
    add_tasks_srv = rospy.ServiceProxy(add_tasks_srv_name, AddTasks)

    if stand_alone_test:
        runner = GCalRoutineRunner(add_tasks_srv, update_wait=1)
    else:
        runner = GCalRoutineRunner(add_tasks_srv)
    if stand_alone_test:
        gcal = GCal(rospy.get_param('~calendar',
                                    'henry.strands%40hanheide.net'),
                    rospy.get_param('~key',
                                    'AIzaSyC1rqV2yecWwV0eLgmoQH7m7PdLNX1p6a0'),
                    update_wait=5,
                    add_cb=runner.add_task, remove_cb=runner.remove_task)
    else:
        gcal = GCal(rospy.get_param('~calendar',
                                    'henry.strands%40hanheide.net'),
                    rospy.get_param('~key',
                                    'AIzaSyC1rqV2yecWwV0eLgmoQH7m7PdLNX1p6a0'),
                    add_cb=runner.add_task, remove_cb=runner.remove_task)
    added = []
    removed = []
    gcal.update(added, removed)
    if stand_alone_test:
        gcal.shift_to_now()
    gcal.start_worker()

    rospy.spin()
