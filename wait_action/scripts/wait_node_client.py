#!/usr/bin/env python

import rospy
import actionlib
from wait_action.msg import *


if __name__ == '__main__':

    rospy.init_node("wait_node_client")
    
    # wait for simulated time to kick in as rospy.get_rostime() is 0 until first clock message received
    while not rospy.is_shutdown() and rospy.get_rostime().secs == 0:
        pass

    
    client = actionlib.SimpleActionClient('wait_action', WaitAction)
    client.wait_for_server()

    wait_secs = 5
    
    # wait a duration
    goal = WaitGoal(wait_duration=rospy.Duration(wait_secs))
    client.send_goal(goal)
    client.wait_for_result()

    # wait until a specific time (the time in wait_secs time)
    goal = WaitGoal(wait_until=(rospy.get_rostime() + rospy.Duration(wait_secs)))
    client.send_goal(goal)
    client.wait_for_result()

    # wait forever (or a really long time at least)
    goal = WaitGoal()
    client.send_goal(goal)
    client.wait_for_result()
