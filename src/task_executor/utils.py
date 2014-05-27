import rospy
import actionlib 
from std_msgs.msg import String
from strands_executive_msgs.msg import Task
from task_executor.msg import *
from topological_navigation.msg import GotoNodeAction, GotoNodeResult
from strands_navigation_msgs.msg import NavStatistics, MonitoredNavigationAction, MonitoredNavigationResult, MonitoredNavigationGoal
from actionlib_msgs.msg import GoalStatus

class TestTaskAction(object):
    """ 
    Creates the action servers the example tasks required.
    """
    def __init__(self, expected_action_duration=rospy.Duration(1), expected_drive_duration=rospy.Duration(20)):
        self.expected_action_duration = expected_action_duration
        self.expected_drive_duration = expected_drive_duration
        self.nav_result   =  GotoNodeResult()    
        self.mon_nav_result =  MonitoredNavigationResult()    
        self.nav_server = actionlib.SimpleActionServer('topological_navigation', GotoNodeAction, execute_cb = self.nav_callback, auto_start = False)
        self.mon_nav_server = actionlib.SimpleActionServer('monitored_navigation', MonitoredNavigationAction, execute_cb = self.mon_nav_callback, auto_start = False)

        # self.task_server = actionlib.SimpleActionServer('test_task', TestExecutionAction, execute_cb = self.execute, auto_start = False)        
        self.cn_pub = rospy.Publisher('/current_node', String)
        self.cl_pub = rospy.Publisher('/closest_node', String)
        self.cn = 'WayPoint1'
       
        
    def start(self):
        self.nav_server.start()         
        self.mon_nav_server.start()         
        # self.task_server.start() 
        while not rospy.is_shutdown():
             self.cn_pub.publish(String(self.cn))
             self.cl_pub.publish(String(self.cn))
             rospy.sleep(1)

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

    def mon_nav_callback(self, goal):
        print 'called with mon_nav goal'
        target = rospy.get_rostime() + self.expected_drive_duration

        # take some time to get there
        while not rospy.is_shutdown() and rospy.get_rostime() < target and not self.mon_nav_server.is_preempt_requested():
            rospy.sleep(0.1)           

        if self.mon_nav_server.is_preempt_requested():
            print "done mon_nav preempted"
            self.mon_nav_result.sm_outcome = MonitoredNavigationResult.PREEMPTED
            self.mon_nav_server.set_preempted(self.mon_nav_result)
        else:
            print "done mon_nav normal"                         
            self.mon_nav_result.sm_outcome = MonitoredNavigationResult.SUCCEEDED       
            self.mon_nav_server.set_succeeded(self.mon_nav_result)

        print "mon_nav complete" 
        

    def nav_callback(self, goal):
        print 'called with nav goal %s'%goal.target

        self.mon_nav_action_client = actionlib.SimpleActionClient('monitored_navigation', MonitoredNavigationAction)
        rospy.loginfo('waiting for mon nav server')
        self.mon_nav_action_client.wait_for_server()
        rospy.loginfo('done')

        mon_goal = MonitoredNavigationGoal()
        self.mon_nav_action_client.send_goal(mon_goal)
        self.mon_nav_action_client.wait_for_result(rospy.Duration(5))

        # wait for completion or prempt
        while not rospy.is_shutdown() and self.mon_nav_action_client.get_state() == GoalStatus.ACTIVE and not self.nav_server.is_preempt_requested():
            self.mon_nav_action_client.wait_for_result(rospy.Duration(0.1))

        if self.nav_server.is_preempt_requested():
            print "done preempted"
            self.mon_nav_action_client.cancel_all_goals()            
            self.nav_result.success = False
            self.nav_server.set_preempted(self.nav_result)
        elif self.mon_nav_action_client.get_state() != GoalStatus.SUCCEEDED:
            print "done mon not SUCCEEDED"            
            self.nav_result.success = False
            self.nav_server.set_aborted(self.nav_result)
        else:
            print "done normal"     
            self.cn = goal.target            
            self.nav_result.success = True       
            self.nav_server.set_succeeded(self.nav_result)
        


        print "nav complete" 