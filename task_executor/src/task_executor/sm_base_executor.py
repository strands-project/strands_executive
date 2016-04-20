#!/usr/bin/env python

import rospy
from strands_executive_msgs.msg import Task, TaskEvent
from strands_executive_msgs.srv import *
import mongodb_store.util as dc_util
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from mongodb_store.message_store import MessageStoreProxy
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import String
from task_executor.base_executor import BaseTaskExecutor
from smach_ros import SimpleActionState, IntrospectionServer
import smach
import threading

class ExecutorState(smach.State):
    def __init__(self, task_executor, outcomes):
        super(ExecutorState , self ).__init__(outcomes=outcomes, input_keys=['task'])        
        self.executor = task_executor

class TimerState(smach.State):
    def __init__(self, duration, pause_secs=1, can_finish=None):
        super(TimerState , self ).__init__(outcomes=['succeeded', 'preempted', 'aborted'])        
        self.duration = duration
        self.pause = rospy.Duration(pause_secs)
        if can_finish is None:
            self.can_finish = lambda: True
        else:
            self.can_finish = can_finish

    def execute(self, userdata):
        now = rospy.get_rostime()
        target = now + self.duration
        while now < target and not self.preempt_requested() and not rospy.is_shutdown():            
            rospy.sleep(self.pause)
            now = rospy.get_rostime()

        # now block if can_finish is not true
        while not self.can_finish() and not self.preempt_requested() and not rospy.is_shutdown():             
            rospy.sleep(self.pause)
            rospy.loginfo('Waiting for signal that I can finish after timeout')

        if rospy.is_shutdown():
            return 'aborted'
        elif self.preempt_requested():
            return 'preempted'
        else:
            return 'succeeded'




class TaskFailed(ExecutorState):
    def __init__(self, task_executor):
        ExecutorState.__init__(self, task_executor, outcomes=['aborted'])        

    def execute(self, userdata):
        rospy.logwarn('Task failed')
        completed = userdata.task
        self.executor.active_tasks = []
        self.executor.log_task_event(userdata.task, TaskEvent.TASK_FAILED, rospy.get_rostime())        
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
        self.executor.active_tasks = []
        self.executor.log_task_event(userdata.task, TaskEvent.TASK_SUCCEEDED, rospy.get_rostime())

        self.executor.task_succeeded(completed)
        rospy.loginfo('Execution of task %s succeeded' % userdata.task.task_id)
        return 'succeeded'              


class TaskCancelled(ExecutorState):
    def __init__(self, task_executor):
        ExecutorState.__init__(self, task_executor, outcomes=['preempted'])        

    def execute(self, userdata):
        rospy.loginfo('Execution of task %s was cancelled' % userdata.task.task_id)
        self.executor.log_task_event(userdata.task, TaskEvent.TASK_PREEMPTED, rospy.get_rostime())
        # it could be that a delayed cancellation signal causes this to get called out of turn, so check that this is the correct cancellation
        if len(self.executor.active_tasks) > 0 and self.executor.active_tasks[0].task_id == userdata.task.task_id:
            completed = userdata.task
            self.executor.active_tasks = []            
            self.executor.task_failed(completed)        
        return 'preempted'


class TaskInitialisation(ExecutorState):
    def __init__(self, task_executor):
        ExecutorState.__init__(self, task_executor, outcomes=['succeeded'])


    def execute(self, userdata):
        self.executor.log_task_event(userdata.task, TaskEvent.TASK_STARTED, rospy.get_rostime())
        self.executor.prepare_task(userdata.task)
        return 'succeeded'

class AbstractTaskExecutor(BaseTaskExecutor):
    """ Provides a base executor using Smach """
    # These can be implemented by sub classes to provide hooks into the execution system


    def __init__(self):
        super( AbstractTaskExecutor, self ).__init__()
        self.task_sm = None
        self.smach_thread = None
        self.sis = None
        self.join_lock = threading.Lock()


    def reset_sm(self):

        # If introspection is running, stop it
        if self.sis is not None:
            self.sis.stop()

        # If the smach_tread is running then we may have started executing prior to the sm winding up fully, or there is something blocking there        
        timeout_secs = 15
        if not self.join_smach_thread(timeout_secs):
            rospy.logerr('SMACH thread has not shut down properly. Continuing with execution regardless')


        self.task_sm = None
        self.smach_thread = None
        self.sis = None

    def nav_start_cb(self, userdata, initial_states):
        task = userdata.task
        rospy.loginfo('Nav started to %s for task %s, action %s' % (task.start_node_id, task.task_id, task.action))
        self.log_task_event(task, TaskEvent.NAVIGATION_STARTED, rospy.get_rostime())                        

        
        
        
        
        
        
        
    def nav_termination_cb(self, userdata, terminal_states, container_outcome):
        rospy.loginfo('Nav terminated with outcome %s' % container_outcome)
        if container_outcome == 'succeeded':
            self.log_task_event(userdata.task, TaskEvent.NAVIGATION_SUCCEEDED, rospy.get_rostime())                        
        elif container_outcome == 'preempted':
            self.log_task_event(userdata.task, TaskEvent.NAVIGATION_PREEMPTED, rospy.get_rostime())                        
        else:
            self.log_task_event(userdata.task, TaskEvent.NAVIGATION_FAILED, rospy.get_rostime())                        

    def action_start_cb(self, userdata, initial_states):
        task = userdata.task
        rospy.loginfo('Action %s started for task %s' % (task.action, task.task_id))
        self.log_task_event(task, TaskEvent.EXECUTION_STARTED, rospy.get_rostime())
        
    def outcome_cb(self, outcome_map):
        if outcome_map['MONITORED'] == 'succeeded':
            return 'succeeded'
        if outcome_map["MONITORED"] == "aborted" or outcome_map["MONITORING"] == "aborted":
            return "aborted"
        if outcome_map["MONITORING"] == "succeeded":
            return "aborted"
        if outcome_map["MONITORED"] == "preempted" or outcome_map["MONITORING"] == "preempted":
            return "preempted"

            
            ## outcome map for monitored states
            #concurrence_outcome_map =  {
                                ## if the monitored state succeeds the concurrence succeeds
                                #'succeeded' : {'MONITORED':'succeeded'},
                                ## if either abort, then the concurrence returns this
                                #'aborted' : {'MONITORED':'aborted'},
                                #'aborted' : {'MONITORING':'aborted'},
                                ## if the monitoring state succeeds the we abort as nav timed out
                                #'aborted' : {'MONITORING':'succeeded'},
                                ## if both were preempted then we were prempted overall
                                #'preempted' : {'MONITORED':'preempted', 'MONITORING':'preempted'}
                                 #}
            

    def action_termination_cb(self, userdata, terminal_states, container_outcome):
        rospy.loginfo('Action terminated with outcome %s' % container_outcome)
        if container_outcome == 'succeeded':
            self.log_task_event(userdata.task, TaskEvent.EXECUTION_SUCCEEDED, rospy.get_rostime())                        
        elif container_outcome == 'preempted':
            self.log_task_event(userdata.task, TaskEvent.EXECUTION_PREEMPTED, rospy.get_rostime())                        
        else:
            self.log_task_event(userdata.task, TaskEvent.EXECUTION_FAILED, rospy.get_rostime())                        

    def execute_task(self, task):
        """ Called to trigger execution of given task. """

        # clean up from last execution
        self.reset_sm()

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



            # when one child terminates, preempt the others
            concurrence_child_term_cb = lambda so: True
            

            # navigation action
            if task.start_node_id != '':

                nav_transitions = {'preempted':'TASK_CANCELLED', 'aborted':'TASK_FAILED'}

                # if no action then nav success leads to task success
                if task.action == '':
                    nav_transitions['succeeded'] = 'TASK_SUCCEEDED'
                # else move on to execution
                else:
                    nav_transitions['succeeded'] = 'TASK_EXECUTION'


                # create a concurrence which monitors execution time
                nav_concurrence = smach.Concurrence(outcomes=['succeeded', 'preempted', 'aborted'],
                                        default_outcome='preempted',
                                        outcome_cb=self.outcome_cb,
                                        child_termination_cb=concurrence_child_term_cb,
                                        # give states 30 seconds to service a request to shut down
                                        termination_timeout = 30)
                # register callback for logging
                nav_concurrence.register_start_cb(self.nav_start_cb)
                nav_concurrence.register_termination_cb(self.nav_termination_cb)
                nav_concurrence.userdata.task = task

                with nav_concurrence:

                    if self.nav_service == BaseTaskExecutor.TOPOLOGICAL_NAV:
                        from topological_navigation.msg import GotoNodeAction, GotoNodeGoal
                        # where we want to go
                        nav_goal = GotoNodeGoal(target = task.start_node_id)
                        nav_action_clz = GotoNodeAction                        
                        nav_action_name = 'topological_navigation'
                    elif self.nav_service == BaseTaskExecutor.MDP_NAV:
                        from strands_executive_msgs.msg import ExecutePolicyAction, ExecutePolicyGoal
                        nav_goal = ExecutePolicyGoal(task_type=ExecutePolicyGoal.GOTO_WAYPOINT, target_id=task.start_node_id)
                        nav_action_clz = ExecutePolicyAction
                        nav_action_name = 'mdp_plan_exec/execute_policy'
                    else:
                        raise RuntimeError('Unknown nav service: %s'% self.nav_service)

                    # let nav run for 2.0 times the length it usually takes before terminating
                    duration_multiplier = 2.0
                    if rospy.get_param('relaxed_nav', False):
                        duration_multiplier = 50

                    # all navigation actions should get at least 30 seconds
                    monitor_duration = max(self.expected_navigation_duration_now(task.start_node_id) * duration_multiplier, rospy.Duration(30))

                    smach.Concurrence.add('MONITORED',
                                                SimpleActionState(nav_action_name,
                                                nav_action_clz,
                                                goal=nav_goal))
                    smach.Concurrence.add('MONITORING', TimerState(duration=monitor_duration))

                smach.StateMachine.add('TASK_NAVIGATION', nav_concurrence, transitions=nav_transitions)

            # actual task action
            if task.action != '':
                try:
                    (action_string, goal_string) = self.get_task_types(task.action)
                    action_clz = dc_util.load_class(dc_util.type_to_class_string(action_string))
                    goal_clz = dc_util.load_class(dc_util.type_to_class_string(goal_string))

                    argument_list = self.get_arguments(task.arguments)
                    goal = goal_clz(*argument_list)         

                    # create a concurrence which monitors execution time along with doing the execution
                    action_concurrence = smach.Concurrence(outcomes=['succeeded', 'preempted', 'aborted'],
                                            default_outcome='preempted',
                                            outcome_cb=self.outcome_cb,
                                            child_termination_cb=concurrence_child_term_cb,
                                            # give states 30 seconds to service a request to shut down
                                            termination_timeout = 30)

                    # register callback for logging
                    action_concurrence.register_start_cb(self.action_start_cb)
                    action_concurrence.register_termination_cb(self.action_termination_cb)
                    action_concurrence.userdata.task = task

                    with action_concurrence:
                        smach.Concurrence.add('MONITORED', SimpleActionState(task.action, action_clz, goal=goal))
                        wiggle_room = rospy.Duration(30)

                        # this prevents the task being interupted if it runs out of time but should still run
                        def task_can_be_interrupted():
                            return self.is_task_interruptible(task)

                        smach.Concurrence.add('MONITORING', TimerState(duration=(task.max_duration+wiggle_room), can_finish=task_can_be_interrupted))

                    smach.StateMachine.add('TASK_EXECUTION',
                                    action_concurrence,
                                    transitions={'preempted':'TASK_CANCELLED', 'aborted':'TASK_FAILED', 'succeeded':'TASK_SUCCEEDED'})

                except Exception, e:
                    rospy.logwarn('Caught exception when creating the task to execute: %s' % e)
                    self.task_failed(task)
                    return 
                                
        self.task_sm.set_initial_state(['TASK_INITIALISATION'])

        self.sis = IntrospectionServer('task_executor', self.task_sm, '/TASK_ROOT')
        self.sis.start()

        self.smach_thread = threading.Thread(target=self.task_sm.execute)
        self.smach_thread.start()

    def join_smach_thread(self, timeout):
        """ Returns true of the smach_thread has been terminated successfully """

        self.join_lock.acquire()

        successfully_joined = True

        # if we need to join
        if self.smach_thread is not None and self.smach_thread.is_alive():
            self.smach_thread.join(timeout)
            if self.smach_thread is not None:
                successfully_joined = not self.smach_thread.is_alive()
            
        self.join_lock.release()

        return successfully_joined

    def cancel_active_task(self):
        preempt_timeout_secs = 60
        if self.task_sm is not None:
            rospy.loginfo('Requesting preempt on state machine in state %s' % self.task_sm.get_active_states())
            self.task_sm.request_preempt()
            rospy.loginfo('Waiting for exit')
            
            if not self.join_smach_thread(preempt_timeout_secs):
                rospy.logerr('Task action or navigation did not preempt after %s seconds. State at end was %s' % (preempt_timeout_secs, self.task_sm.get_active_states()))
                # manually notify completeness in this case
                if len(self.active_tasks) > 0:
                    completed = self.active_tasks[0]
                    self.active_task = []                
                    self.task_failed(completed)        
            else:                
                rospy.loginfo('And relax')
            
