#!/usr/bin/env python

import rospy
from rosgraph_msgs.msg import Clock



if __name__ == '__main__':

    # make sure we are not using sim time
    rospy.set_param('use_sim_time', False)

    rospy.init_node("sim_clock", log_level=rospy.INFO)
 
    # make sure everyone else started after no uses sim time
    rospy.set_param('use_sim_time', True)

    clock_pub = rospy.Publisher('/clock', Clock)

    # how much to increment the simulated clock per second, default 1 minute per second
    seconds_per_second = rospy.Duration(rospy.get_param('sim_time_step', 60))
    rospy.set_param('sim_time_step', seconds_per_second.to_sec())



    # how often to update clock
    rate = rospy.Rate(1)

    # start value
    clock_msg = Clock(clock=rospy.get_rostime())

    while not rospy.is_shutdown():

        # publish time
        clock_pub.publish(clock_msg)

        # update time
        clock_msg.clock += seconds_per_second

        rate.sleep()

    # reset so sim time is not left on
    rospy.set_param('use_sim_time', False)