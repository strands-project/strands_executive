import rospy
import actionlib 
from task_executor.msg import *

class TestTaskAction(object):
    """ 
    Creates the action servers the example tasks require.
    """
    def __init__(self, expected_action_duration=rospy.Duration(1)):
        self.expected_action_duration = expected_action_duration
        self.task_server = actionlib.SimpleActionServer('test_task', TestExecutionAction, execute_cb = self.execute, auto_start = False)        
        
    def execute(self, goal):
        print 'called with goal %s'%goal.some_goal_string
        target = rospy.get_rostime() + self.expected_action_duration

        while not rospy.is_shutdown() and rospy.get_rostime() < target and not self.task_server.is_preempt_requested():
            rospy.sleep(0.1)       

        if self.task_server.is_preempt_requested():
            print "done preempted"
            self.task_server.set_preempted()
        else:
            print "done normal"            
            self.task_server.set_succeeded()

