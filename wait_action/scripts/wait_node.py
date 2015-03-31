#!/usr/bin/env python

import rospy
from wait_action.msg import WaitAction
from datetime import *
from std_srvs.srv import Empty, EmptyResponse
from strands_executive_msgs import task_utils
from strands_executive_msgs.abstract_task_server import AbstractTaskServer

class WaitServer(AbstractTaskServer):
    def __init__(self):
        super(WaitServer, self).__init__('wait_action', action_type=WaitAction)

    def end_wait(self, req):

        if self.server.is_active():
            rospy.loginfo("Preempting sleep")
            self.server.preempt_request = True
            self.server.set_preempted()
        return EmptyResponse()

    def create(self, req):
        t = super(WaitServer, self).create(req)
        task_utils.add_time_argument(t, rospy.Time())
        task_utils.add_duration_argument(t, t.max_duration)
        return t

    def execute(self, goal):
        end_wait_srv = rospy.Service('/wait_action/end_wait',
                                     Empty, self.end_wait)

        now = rospy.get_rostime()
        target = goal.wait_until
        if target.secs == 0:
            target = now + goal.wait_duration

        if target == now:
            # 72 hours
            a_really_long_time = rospy.Duration(60 * 60 * 24 * 3)
            target = now + a_really_long_time
            # rospy.loginfo("waiting a really long time")


        rospy.loginfo("target wait time: %s" \
            % datetime.fromtimestamp(target.secs))	

        # how often to provide feedback 
        feedback_secs = 5.0
        feedback = WaitFeedback()
        # how long to wait
        wait_duration_secs = target.secs - now.secs

        # the rate to publish feedback at/check time at
        r = rospy.Rate(1.0/feedback_secs)

        count = 1.0
        feedback_steps = wait_duration_secs/feedback_secs
        while not rospy.is_shutdown() and not self.server.is_preempt_requested() and rospy.get_rostime() < target:
            feedback.percent_complete = count/feedback_steps
            self.server.publish_feedback(feedback)
            count += 1
            r.sleep()

        rospy.loginfo("waited until: %s" % datetime.fromtimestamp(rospy.get_rostime().secs))



        end_wait_srv.shutdown()

        if self.server.is_preempt_requested():
            self.server.set_preempted()
        else:
            self.server.set_succeeded()


if __name__ == '__main__':
    rospy.init_node("wait_node")

# wait for simulated time to kick in as rospy.get_rostime() is 0 until first clock message received
    while not rospy.is_shutdown() and rospy.get_rostime().secs == 0:
        pass

    waiter = WaitServer()

    rospy.spin()
