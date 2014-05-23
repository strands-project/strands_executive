#!/usr/bin/env python

import rospy
from strands_executive_msgs.msg import Task, TaskEvent
from strands_executive_msgs.srv import *
import ros_datacentre.util as dc_util
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from ros_datacentre.message_store import MessageStoreProxy
#from topological_navigation.msg import GotoNodeAction, GotoNodeGoal
from strands_executive_msgs.msg import ExecutePolicyAction, ExecutePolicyGoal
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import String
from task_executor.base_executor import BaseTaskExecutor
from smach_ros import SimpleActionState
import smach


class ExecutorState(smach.State):
    def __init__(self, task_executor, outcomes):
        super(ExecutorState , self ).__init__(outcomes=outcomes, input_keys=['task'])        
        self.executor = task_executor


class NavigationAborted(ExecutorState):
    def __init__(self, task_executor):
        ExecutorState.__init__(self, task_executor, outcomes=['aborted'])        

    def execute(self, userdata):
        rospy.logwarn('Navigation aborted for task')
        return 'aborted'

class NavigationPreempted(ExecutorState):
    def __init__(self, task_executor):
        ExecutorState.__init__(self, task_executor, outcomes=['preempted'])        

    def execute(self, userdata):
        rospy.logwarn('Navigation was preempted for task')
        return 'preempted'


class TaskFailed(ExecutorState):
    def __init__(self, task_executor):
        ExecutorState.__init__(self, task_executor, outcomes=['aborted'])        

    def execute(self, userdata):
        rospy.logwarn('Task failed')
        return 'aborted'

class TaskSucceeded(ExecutorState):
    def __init__(self, task_executor):
        ExecutorState.__init__(self, task_executor, outcomes=['succeeded'])        

    def execute(self, userdata):
        now = rospy.get_rostime()
        # do bookkeeping before causing update
        completed = self.active_task
        self.executor.active_task = None
        self.executor.active_task_id = Task.NO_TASK
        self.executor.task_succeeded(completed)
        self.executor.log_task_event(completed, TaskEvent.TASK_FINISHED, now) 
        return 'succeeded'              


class TaskCancelled(ExecutorState):
    def __init__(self, task_executor):
        ExecutorState.__init__(self, task_executor, outcomes=['preempted'])        

    def execute(self, userdata):
        rospy.logwarn('Task cancelled')
        return 'preempted'


class TaskInitialisation(ExecutorState):
    def __init__(self, task_executor):
        ExecutorState.__init__(self, task_executor, outcomes=['succeeded'])


    def execute(self, userdata):
        self.executor.prepare_task(userdata.task)
        return 'succeeded'

class AbstractTaskExecutor(BaseTaskExecutor):
    """ Provides a base executor using Smach """
    # These can be implemented by sub classes to provide hooks into the execution system


    def __init__(self):
        super( AbstractTaskExecutor, self ).__init__()

    def execute_task(self, task):
        """ Called to trigger execution of given task. """

        # Create the state machine necessary to execution this task        
        self.task_sm = smach.StateMachine(['succeeded','aborted','preempted'])
        self.task_sm.userdata.task = task

        with self.task_sm:

            # Initialise task data
            init_transition = {}
            if task.start_node_id != '':
                init_transition = {'succeeded': 'TASK_NAVIGATION'}
            else:
                init_transition = {'succeeded': 'succeeded'}


            smach.StateMachine.add('TASK_INITIALISATION', TaskInitialisation(self), transitions=init_transition, remapping={'task':'task'})
            
            # Final task outcomes
            smach.StateMachine.add('TASK_SUCCEEDED', TaskSucceeded(self), transitions={'succeeded':'succeeded'})
            smach.StateMachine.add('TASK_CANCELLED', TaskCancelled(self), transitions={'preempted':'preempted'})
            smach.StateMachine.add('TASK_FAILED', TaskFailed(self), transitions={'aborted':'aborted'})

            # # Non-success navigation states
            smach.StateMachine.add('NAVIGATION_PREEMMPTED', NavigationPreempted(self), transitions={'preempted':'TASK_CANCELLED'})
            smach.StateMachine.add('NAVIGATION_ABORTED', NavigationAborted(self), transitions={'aborted':'TASK_FAILED'})

            # navigation action
            if task.start_node_id != '':

                nav_transitions = {'preempted':'NAVIGATION_PREEMMPTED', 'aborted':'NAVIGATION_ABORTED'}

                # if no action then nav success leads to task success
                if task.action = '':
                    nav_transitions['succeeded'] = 'TASK_SUCCEEDED'
                # else move on to execution
                else:
                    nav_transitions['succeeded'] = 'succeeded'

                nav_goal = ExecutePolicyGoal(task_type=ExecutePolicyGoal.GOTO_WAYPOINT, target_id=task.start_node_id, time_of_day='all_day')
                smach.StateMachine.add('TASK_NAVIGATION',
                                SimpleActionState('mdp_plan_exec/execute_policy',
                                    ExecutePolicyAction,
                                    goal=nav_goal),
                                transitions=nav_transitions)


        self.task_sm.set_initial_state(['TASK_INITIALISATION'])

        self.task_sm.execute()


