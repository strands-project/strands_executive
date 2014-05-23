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
import threading

class ExecutorState(smach.State):
    def __init__(self, task_executor, outcomes):
        super(ExecutorState , self ).__init__(outcomes=outcomes, input_keys=['task'])        
        self.executor = task_executor


class TaskFailed(ExecutorState):
    def __init__(self, task_executor):
        ExecutorState.__init__(self, task_executor, outcomes=['aborted'])        

    def execute(self, userdata):
        rospy.logwarn('Task failed')
        completed = userdata.task
        self.executor.active_task = None
        
        self.executor.task_failed(completed)
        rospy.loginfo('Execution of task %s failed' % userdata.task.task_id)
        return 'aborted'

class TaskSucceeded(ExecutorState):
    def __init__(self, task_executor):
        ExecutorState.__init__(self, task_executor, outcomes=['succeeded'])        

    def execute(self, userdata):
        now = rospy.get_rostime()
        # do bookkeeping before causing update
        completed = userdata.task
        self.executor.active_task = None
        
        self.executor.task_succeeded(completed)
        rospy.loginfo('Execution of task %s succeeded' % userdata.task.task_id)
        return 'succeeded'              


class TaskCancelled(ExecutorState):
    def __init__(self, task_executor):
        ExecutorState.__init__(self, task_executor, outcomes=['preempted'])        

    def execute(self, userdata):
        rospy.loginfo('Execution of task %s was cancelled' % userdata.task.task_id)

        # it could be that a delayed cancellation signal causes this to get called out of turn, so check that this is the correct cancellation
        if self.executor.active_task is not None and self.executor.active_task.task_id == userdata.task.task_id:
            completed = userdata.task
            self.executor.active_task = None            
            self.executor.task_failed(completed)        
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
        self.reset_sm()

    def reset_sm(self):
        self.task_sm = None
        self.smach_thread = None

    def execute_task(self, task):
        """ Called to trigger execution of given task. """

        rospy.loginfo('Execution of task %s was requested' % task.task_id)

        # Create the state machine necessary to execution this task        
        self.task_sm = smach.StateMachine(['succeeded','aborted','preempted'])
        self.task_sm.userdata.task = task

        with self.task_sm:

            # Initialise task data
            init_transition = {}
            if task.start_node_id != '':
                init_transition = {'succeeded': 'TASK_NAVIGATION'}
            else:
                init_transition = {'succeeded': 'TASK_EXECUTION'}


            smach.StateMachine.add('TASK_INITIALISATION', TaskInitialisation(self), transitions=init_transition)
            
            # Final task outcomes
            smach.StateMachine.add('TASK_SUCCEEDED', TaskSucceeded(self), transitions={'succeeded':'succeeded'})
            smach.StateMachine.add('TASK_CANCELLED', TaskCancelled(self), transitions={'preempted':'preempted'})
            smach.StateMachine.add('TASK_FAILED', TaskFailed(self), transitions={'aborted':'aborted'})

            # navigation action
            if task.start_node_id != '':

                nav_transitions = {'preempted':'TASK_CANCELLED', 'aborted':'TASK_FAILED'}

                # if no action then nav success leads to task success
                if task.action == '':
                    nav_transitions['succeeded'] = 'TASK_SUCCEEDED'
                # else move on to execution
                else:
                    nav_transitions['succeeded'] = 'TASK_EXECUTION'

                nav_goal = ExecutePolicyGoal(task_type=ExecutePolicyGoal.GOTO_WAYPOINT, target_id=task.start_node_id, time_of_day='all_day')
                smach.StateMachine.add('TASK_NAVIGATION',
                                SimpleActionState('mdp_plan_exec/execute_policy',
                                    ExecutePolicyAction,
                                    goal=nav_goal),
                                transitions=nav_transitions)

            # actual task action
            if task.action != '':
                (action_string, goal_string) = self.get_task_types(task.action)
                action_clz = dc_util.load_class(dc_util.type_to_class_string(action_string))
                goal_clz = dc_util.load_class(dc_util.type_to_class_string(goal_string))

                argument_list = self.get_arguments(task.arguments)
                goal = goal_clz(*argument_list)         

                smach.StateMachine.add('TASK_EXECUTION',
                                SimpleActionState(task.action,
                                    action_clz,
                                    goal=goal),
                                transitions={'preempted':'TASK_CANCELLED', 'aborted':'TASK_FAILED', 'succeeded':'TASK_SUCCEEDED'})




        self.task_sm.set_initial_state(['TASK_INITIALISATION'])

        self.smach_thread = threading.Thread(target=self.task_sm.execute)
        self.smach_thread.start()

    def cancel_active_task(self):
        preempt_timeout_secs = 10
        if self.task_sm is not None:
            rospy.loginfo('Requesting preempt on state machine in state %s' % self.task_sm.get_active_states())
            self.task_sm.request_preempt()
            rospy.loginfo('Waiting for exit')
            self.smach_thread.join(preempt_timeout_secs)
            if self.smach_thread.is_alive():
                rospy.logerr('Task action or navigation did not preempt after %s seconds. State at end was %s' % (preempt_timeout_secs, self.task_sm.get_active_states()))
                # manually notify completeness in this case
                completed = self.active_task
                self.active_task = None                
                self.task_failed(completed)        
            else:                
                rospy.loginfo('And relax')
            self.reset_sm()
