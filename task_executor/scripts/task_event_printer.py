#!/usr/bin/env python

import rospy
from task_executor import task_routine, task_query
from strands_executive_msgs.msg import *
from random import random
from dateutil.tz import *
from datetime import *

def on_event(task_event):
    rostime_now = rospy.get_rostime()
    now = datetime.fromtimestamp(rostime_now.to_sec(), tzlocal())
    rospy.loginfo('task %s\t%s\t%s\t%s\t%s' % 
        (task_event.task.task_id, task_event.task.action, task_event.task.start_node_id, task_query.task_event_string(task_event.event), now.strftime('%d/%m/%y %H:%M:%S')))


if __name__ == '__main__':
    rospy.init_node('task_event_printer')
    rospy.Subscriber('/task_executor/events', TaskEvent, on_event)
    rospy.spin()