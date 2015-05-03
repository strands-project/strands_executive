#!/usr/bin/env python

import rospy
from strands_executive_msgs.msg import *
from random import random
from dateutil.tz import *
from datetime import *

from task_executor import task_query

def callback(data):
    print task_query.format_event(data)

if __name__ == '__main__':
    rospy.init_node('task_status_printer')
    rospy.Subscriber('task_executor/events', TaskEvent, callback)
 
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()