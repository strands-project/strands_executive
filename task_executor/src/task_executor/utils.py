import rospy
import actionlib
from std_msgs.msg import String
from strands_executive_msgs.msg import Task
from task_executor.msg import *
from topological_navigation.msg import GotoNodeAction, GotoNodeResult


class TestTaskAction(object):
    """ 
    Creates the action servers the example tasks required.
    """
    def __init__(self, expected_action_duration=rospy.Duration(1), expected_drive_duration=rospy.Duration(1)):
        self.expected_action_duration = expected_action_duration
        self.expected_drive_duration = expected_drive_duration
        self.result   =  GotoNodeResult()
        self.nav_server = actionlib.SimpleActionServer('topological_navigation', GotoNodeAction, execute_cb = self.nav_callback, auto_start = False)
        self.nav_server.start() 
        self.task_server = actionlib.SimpleActionServer('test_task', TestExecutionAction, execute_cb = self.execute, auto_start = False)
        self.task_server.start() 
        self.cn_pub = rospy.Publisher('/current_node', String, latch=True)
        self.cn_pub.publish(String('WayPoint1'))
        

    def execute(self, goal):
        print 'called with goal %s'%goal.some_goal_string
        target = rospy.get_rostime() + self.expected_action_duration

        while not rospy.is_shutdown() and rospy.get_rostime() < target and not self.task_server.is_preempt_requested():
            rospy.sleep(20)       
        



        if self.task_server.is_preempt_requested():
            print "done preempted"
            self.task_server.set_preempted()
        else:
            print "done normal"            
            self.task_server.set_succeeded()
        

    def nav_callback(self, goal):
        print 'called with nav goal %s'%goal.target
        target = rospy.get_rostime() + self.expected_drive_duration

        while not rospy.is_shutdown() and rospy.get_rostime() < target and not self.nav_server.is_preempt_requested():
            rospy.sleep(20)       
        
        self.cn_pub.publish(String(goal.target))

        if self.nav_server.is_preempt_requested():
            print "done preempted"
            self.result.success = False
            self.nav_server.set_preempted(self.result)
        else:
            print "done normal"     
            self.result.success = True       
            self.nav_server.set_succeeded(self.result)

