#!/usr/bin/env python

import rospy
from wait_action.msg import WaitAction
from wait_action.msg import WaitFeedback
from datetime import *
from std_srvs.srv import Empty, EmptyResponse
from strands_executive_msgs import task_utils
from strands_executive_msgs.abstract_task_server import AbstractTaskServer


class WaitServer(AbstractTaskServer):
    def __init__(self, interruptible=True, name='wait_action'):
        super(WaitServer, self).__init__(name, action_type=WaitAction,
                                         interruptible=interruptible)
        self.maximise_duration_delta = rospy.Duration(rospy.get_param('~maximise_duration_delta', 0))

    def end_wait(self, req):

        if self.server.is_active():
            rospy.loginfo("Preempting sleep")
            self.server.preempt_request = True
            self.server.set_preempted()
        return EmptyResponse()

    def create(self, req):
        t = super(WaitServer, self).create(req)
        task_utils.add_time_argument(t, rospy.Time())
        if self.maximise_duration_delta > rospy.Duration(0):
            d = (t.end_before - t.start_after) - self.maximise_duration_delta
            task_utils.add_duration_argument(t, d)
            t.max_duration = d
        else:
            task_utils.add_duration_argument(t, t.max_duration)
        return t

    def execute(self, goal):
        end_wait_srv = rospy.Service('/wait_action/end_wait',
                                     Empty, self.end_wait)
        try:
            now = rospy.get_rostime()
            target = goal.wait_until
            if target.secs == 0:
                target = now + goal.wait_duration

            if target == now:
                # 72 hours
                a_really_long_time = rospy.Duration(60 * 60 * 24 * 3)
                target = now + a_really_long_time
                # rospy.loginfo("waiting a really long time")

            rospy.loginfo("target wait time: %s"
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

            rospy.loginfo("waited until: %s"
                          % datetime.fromtimestamp(rospy.get_rostime().secs))
        finally:
            end_wait_srv.shutdown()

        if self.server.is_preempt_requested():
            self.server.set_preempted()
        else:
            self.server.set_succeeded()


if __name__ == '__main__':

    rospy.init_node("wait_node")

    interruptible = rospy.get_param("~interruptible", True)
    # wait for simulated time to kick in as rospy.get_rostime() is 0 until first clock message received
    while not rospy.is_shutdown() and rospy.get_rostime().secs == 0:
        pass

    waiter = WaitServer(interruptible=interruptible, name=rospy.get_name())

    rospy.spin()
