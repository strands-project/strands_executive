#!/usr/bin/env python

import rospy
import actionlib
from strands_executive_msgs.msg import Task
from strands_executive_msgs import task_utils
from task_executor.msg import *
from topological_navigation.msg import GotoNodeAction
from task_executor import task_routine
from datetime import *
from threading import Thread
import ros_datacentre_msgs.srv as dc_srv
from ros_datacentre_msgs.msg import StringPair
import ros_datacentre.util as dc_util
from ros_datacentre.message_store import MessageStoreProxy
from geometry_msgs.msg import Pose, Point, Quaternion
from dateutil.tz import tzlocal



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
        

def dummy_task():
    """ 
    Create an example of a task which we'll copy for other tasks later.
    This is a good example of creating a task with a variety of arguments.
    """
     # need message store to pass objects around
    msg_store = MessageStoreProxy() 

    # get the pose of a named object
    pose_name = "my favourite pose"

    # get the pose if it's there
    message, meta =  msg_store.query_named(pose_name, Pose._type)
    # if it's not there, add it in
    if message == None: 
        message = Pose(Point(0, 1, 2), Quaternion(3, 4,  5, 6))
        pose_id = msg_store.insert_named(pose_name, message)
    else:
        pose_id = meta["_id"]           

    master_task = Task(action='test_task')        
    task_utils.add_string_argument(master_task, 'hello world')
    task_utils.add_object_id_argument(master_task, pose_id, Pose)
    task_utils.add_int_argument(master_task, 24)
    task_utils.add_float_argument(master_task, 63.678)
    return master_task

        

if __name__ == '__main__':

    rospy.init_node("task_routine", log_level=rospy.INFO)
 
    # wait for simulated time to kick in as rospy.get_rostime() is 0 until first clock message received
    while not rospy.is_shutdown() and rospy.get_rostime().secs == 0:
        pass

    localtz = tzlocal()

    start = time(9,00, tzinfo=localtz)
    ten = time(10,00, tzinfo=localtz)
    midday = time(12,00, tzinfo=localtz)
    end = time(17,00, tzinfo=localtz)
    afternoon = (time(14,00, tzinfo=localtz), end)

    # print start
    # print task_routine.time_to_secs(start)
    # print end
    # print task_routine.time_to_secs(end)

    # print datetime.today()
    # last_midnight = datetime.fromordinal(datetime.today().toordinal())
    # print task_routine.unix_time(last_midnight)


    # # some standard intervals
    # today = (start, end)
    # morning = (start, midday)
    # afternoon = (midday, end)

    # start_midnight_timer = Thread(target=delay_to_midnight)    
    # start_midnight_timer.start()

    # rospy.Timer(rospy.Duration(60*60), print_time)


    routine = task_routine.DailyRoutine(start, end)

    task = dummy_task()
    
    
    # all these should except
    # routine.add_task(task, midday, start)
    # routine.add_task(task, start, start)
    # task.expected_duration = 60 * 60 * 12;
    # routine.add_task(task, start, midday)

    task.expected_duration = rospy.Duration(timedelta(seconds=60).total_seconds());

    routine.add_task(task, *afternoon)

    routine._new_day()

    print "off to spin"

    rospy.spin()

