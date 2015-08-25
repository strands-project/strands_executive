#!/usr/bin/env python

import rospy

from routine_behaviours.robot_routine import RobotRoutine

from datetime import time, date, timedelta
from dateutil.tz import tzlocal

from strands_executive_msgs.msg import Task
from strands_executive_msgs import task_utils

def wait_task_at_waypoint(wp):
    # construct task
    task = Task()

    # set action to execute
    task.action = '/wait_action'

    # set how long this should be allowed to run in the worst case
    max_wait_secs = 60
    task.max_duration = rospy.Duration(max_wait_secs)

    task_utils.add_time_argument(task, rospy.Time())
    task_utils.add_duration_argument(task, rospy.Duration(10))

    task.start_node_id = wp
    task.end_node_id = wp

    return task

class ExampleRoutine(RobotRoutine):
    """ Creates a routine which simply visits nodes. """

    def __init__(self, daily_start, daily_end, idle_duration=rospy.Duration(5), charging_point = 'ChargingPoint'):
        # super(PatrolRoutine, self).__init__(daily_start, daily_end)        
        RobotRoutine.__init__(self, daily_start, daily_end, idle_duration=idle_duration, charging_point=charging_point)



    def create_routine(self):
        daily_wps = ['WayPoint6', 'WayPoint3']
        daily_wp_tasks = [wait_task_at_waypoint(wp) for wp in daily_wps]
        self.routine.repeat_every_day(daily_wp_tasks)

        minute_wps = ['WayPoint5']
        minute_wp_tasks = [wait_task_at_waypoint(wp) for wp in minute_wps]
        self.routine.repeat_every_delta(minute_wp_tasks, delta=timedelta(minutes=4))


    def on_idle(self):
        """
            Called when the routine is idle. Default is to trigger travel to the charging. As idleness is determined by the current schedule, if this call doesn't utlimately cause a task schedule to be generated this will be called repeatedly.
        """
        rospy.loginfo('I am idle')    



if __name__ == '__main__':

    rospy.init_node('tutorial_4')

    # start and end times -- all times should be in local timezone
    localtz = tzlocal()
    start = time(8,00, tzinfo=localtz)
    end = time(20,00, tzinfo=localtz)

    # how long to stand idle before doing something
    idle_duration=rospy.Duration(20)

    routine = ExampleRoutine(daily_start=start, daily_end=end, idle_duration=idle_duration)    
     
    routine.create_routine()
 
    routine.start_routine()

    rospy.spin()
