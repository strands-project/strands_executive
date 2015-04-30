import rospy
import actionlib 
from task_executor.msg import *
from datetime import datetime, timedelta
from std_msgs.msg import String

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

    def start(self):
        self.task_server.start()


class CheckTaskActionServer(object):
    """ 
    Creates the action servers the example tasks require.
    """
    def __init__(self, result_fn=None):
        self.current_node = None
        self.result_fn = result_fn

        rospy.Subscriber('/current_node', String, self.update_topological_location, queue_size=2)
        while self.current_node is None and not rospy.is_shutdown():
            rospy.sleep(0.5)

        self.task_server = actionlib.SimpleActionServer('check_task', TaskTestAction, execute_cb = self.execute, auto_start = False)        

    def update_topological_location(self, node_name):
        self.current_node = node_name.data
        
    def execute(self, goal):

        now = rospy.get_rostime()

        result = TaskTestResult()
        result.success = True

        task = goal.task

        if task.start_node_id != self.current_node:
            result.success = False
            rospy.logwarn('Start node incorrect')        

        rospy.loginfo('        now: %s' % rostime_to_python(now))
        rospy.loginfo('start after: %s' % rostime_to_python(task.start_after))
        rospy.loginfo(' end before: %s' % rostime_to_python(task.end_before))

        if now < task.start_after:
            result.success = False
            rospy.logwarn('Task started before window is open')
            rospy.logwarn('        now: %s' % rostime_to_python(now))
            rospy.logwarn('start after: %s' % rostime_to_python(task.start_after))

        if now + task.max_duration > task.end_before:
            result.success = False
            rospy.logwarn('Task could end after window')
            rospy.logwarn('max end time: %s' % rostime_to_python(now + task.max_duration))
            rospy.logwarn('  end before: %s' % rostime_to_python(task.end_before))   

        if self.result_fn is not None:
            self.result_fn(result.success)

        if self.task_server.is_preempt_requested():
            self.task_server.set_preempted(result)
        else:
            self.task_server.set_succeeded(result)

    def start(self):
        self.task_server.start()


def rostime_to_python(rtime):
    return datetime.fromtimestamp(rtime.to_sec())        

def rosduration_to_python(rdur):
    return timedelta(seconds=rdur.to_sec())        


