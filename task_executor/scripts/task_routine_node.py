#!/usr/bin/env python

import rospy
import actionlib
from strands_executive_msgs.msg import Task
from task_executor.msg import *
from topological_navigation.msg import GotoNodeAction
from task_executor import task_routine
from datetime import *
from threading import Thread



def print_time(event):
    # print datetime.fromtimestamp(rospy.get_rostime().secs)
    print datetime.fromtimestamp(event.current_real.secs)

# Timer seems odd
# def midnight_cb(event):
#     print "MIDNIGHT: %s" % datetime.fromtimestamp(event.current_real.secs)
#     if event.last_duration:
#         print event.last_duration

def delay_to_midnight():
    # datetime for upcoming midnight

    while not rospy.is_shutdown():
        # jump to the start of the next day in rostime
        midnight = datetime.fromordinal(datetime.fromtimestamp(rospy.get_rostime().to_sec()).toordinal() + 1)
        print "midnight  %s" % midnight
        midnight_rostime = rospy.Time(task_routine.unix_time(midnight))
        
        now = rospy.get_rostime()
        print "midnight_rostime  %s" % midnight_rostime.to_sec()
        print "             now  %s" % now.to_sec()
        assert midnight_rostime > now
        midnight_delay = midnight_rostime - now
        
        print "   midnight delay %s" % midnight_delay.to_sec()
        # sleep until midnight
        rospy.sleep(midnight_delay)
        # the set a recurring timer for everynight
        print "MIDNIGHT %s" % datetime.fromtimestamp(rospy.get_rostime().to_sec())
        
        

if __name__ == '__main__':

    rospy.init_node("task_routine", log_level=rospy.INFO)
 
    # wait for simulated time to kick in as rospy.get_rostime() is 0 until first clock message received
    while not rospy.is_shutdown() and rospy.get_rostime().secs == 0:
        pass

    start = time(9,00)
    midday = time(12,00)
    end = time(17,00)

    print start
    print task_routine.time_to_secs(start)
    print end
    print task_routine.time_to_secs(end)

    print datetime.today()
    last_midnight = datetime.fromordinal(datetime.today().toordinal())
    print task_routine.unix_time(last_midnight)


    # some standard intervals
    today = (start, end)
    morning = (start, midday)
    afternoon = (midday, end)

    start_midnight_timer = Thread(target=delay_to_midnight)    
    start_midnight_timer.start()

    rospy.Timer(rospy.Duration(60*60), print_time)


    rospy.spin()

