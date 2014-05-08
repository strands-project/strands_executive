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
from std_srvs.srv import Empty
from std_msgs.msg import String


class AbstractTaskExecutor(object):

    # These can be implemented by sub classes to provide hooks into the execution system

    def add_tasks(self, tasks):
        """ Called with new tasks for the executor """

    def start_execution(self):
        """ Called when overall execution should  (re)start """
        pass

    def pause_execution(self):
        """ Called when overall execution should pause """
        pass

    def task_complete(self, task):
        """ Called when the given task has completed execution """
        pass

    def task_succeeded(self, task):
        """ Called when the given task has completed execution successfully """
        self.task_complete(task)

    def task_failed(self, task):
        """ Called when the given task has completed execution but failed """
        self.task_complete(task)


    def task_demanded(self, demanded_task, currently_active_task):
        """ Called when a task is demanded. self.active_task is the demanded task (and is being executed) and previously_active_task was the task that was being executed (which could be None) """
        pass

    def cancel_task(self, task_id):
        """ Called when a request is received to cancel a task. The currently executing one is checked elsewhere. """
        return False

    def clear_schedule(self):
        """ Called to clear all tasks from schedule, with the exception of the currently executing one. """
        pass


    def __init__(self):
        self.task_counter = 1
        self.msg_store = MessageStoreProxy() 
        self.logging_msg_store = MessageStoreProxy(collection='task_events') 
        self.executing = False
        self.active_task = None
        self.active_task_id = Task.NO_TASK
        self.nav_client = None
        self.action_client = None
        self.active_task_completes_by = rospy.get_rostime()
        
<<<<<<< HEAD
        expected_time_srv_name = '/mdp_plan_exec/get_expected_travel_time_to_node'
=======
        expected_time_srv_name = '/mdp_plan_exec/get_expected_travel_time'
>>>>>>> 6a20bbe36a6a677435fc70684ffbf492b494a74f
        rospy.loginfo('Waiting for %s' % expected_time_srv_name)
        rospy.wait_for_service(expected_time_srv_name)
        rospy.loginfo('... and got %s' % expected_time_srv_name)
        self.expected_time = rospy.ServiceProxy(expected_time_srv_name, GetExpectedTravelTime)

        # start with some faked but likely one in case of problems
        self.current_node = 'WayPoint1'
        rospy.Subscriber('/current_node', String, self.update_topological_location)

    def update_topological_location(self, node_name):
        self.current_node = node_name.data


    def advertise_services(self):
        """
        Adverstise ROS services. Only call at the end of constructor to avoid calls during construction.
        """
        # advertise ros services
        for attr in dir(self):
            if attr.endswith("_ros_srv"):
                service=getattr(self, attr)                
                rospy.Service("/task_executor/" + attr[:-8], service.type, service)


    def get_task_types(self, action_name):
        """ 
        Returns the type string related to the action string provided.
        """
        rospy.logdebug("task action provided: %s", action_name)
        topics = rospy.get_published_topics(action_name)
        for [topic, type] in topics:            
            if topic.endswith('feedback'):
                return (type[:-8], type[:-14] + 'Goal')
        raise RuntimeError('No action associated with topic: %s'% action_name)


    def expected_navigation_duration(self, task): 
        et = self.expected_time(start_id=self.current_node, target_id=task.start_node_id,time_of_day="all_day")
        # rospy.loginfo('expected travel time %s' % et.travel_time)
        # allow a bit of time for any transition -- mainly for testing cases
        return rospy.Duration(max(et.travel_time, 10))
        # return rospy.Duration(120)

    def get_active_task_completion_time(self):
        return self.active_task_completes_by

    def execute_task(self, task):
        self.active_task = task
        self.active_task_id = task.task_id               

        now = rospy.get_rostime()
        total_task_duration = self.expected_navigation_duration(task) + task.max_duration
        self.active_task_completes_by = now + total_task_duration

        self.log_task_event(self.active_task, TaskEvent.TASK_STARTED, now)

        if self.active_task.start_node_id != '':                    
            self.start_task_navigation()
        elif self.active_task.action != '':                    
            self.start_task_action()
        else:
            warning = 'Provided task had no start_node_id or action %s' % self.active_task
            rospy.logwarn(warning)
            self.log_task_event(self.active_task, TaskEvent.TASK_COMPLETE, now, warning)

            self.active_task = None
            self.active_task_id = Task.NO_TASK


    def cancel_active_task(self, event):
        """ Cancel any active nav or task action """
        if event:
            rospy.logwarn("Cancelling task that has overrun %s" % self.active_task.task_id)
        else:
            rospy.logwarn("Cancelling task %s" % self.active_task.task_id)

        if self.nav_client is not None and self.nav_client.get_state() == GoalStatus.ACTIVE:
            self.nav_client.cancel_goal()
            self.nav_timeout_timer.shutdown()            
        if self.action_client  is not None and self.action_client.get_state() == GoalStatus.ACTIVE:
            self.action_timeout_timer.shutdown()
            self.action_client.cancel_goal()            


    def cancel_navigation(self, event):
        """ Called on nav timeout """
        rospy.logwarn("Cancelling navigation that has overrun for task %s" % self.active_task.task_id)
        # cancel navigation
        if self.nav_client and self.nav_client.get_state() == GoalStatus.ACTIVE:
            self.nav_client.cancel_goal()

    def start_task_action(self):

        try:
            rospy.loginfo('Starting to execute %s' % self.active_task.action)
            self.log_task_event(self.active_task, TaskEvent.EXECUTION_STARTED, rospy.get_rostime())

            (action_string, goal_string) = self.get_task_types(self.active_task.action)
            action_clz = dc_util.load_class(dc_util.type_to_class_string(action_string))
            goal_clz = dc_util.load_class(dc_util.type_to_class_string(goal_string))

            self.action_client = actionlib.SimpleActionClient(self.active_task.action, action_clz)
            self.action_client.wait_for_server()

            argument_list = self.get_arguments(self.active_task.arguments)

            # print "ARGS:"
            # print argument_list

            goal = goal_clz(*argument_list)         

            rospy.logdebug('Sending goal to %s' % self.active_task.action)
            self.action_client.send_goal(goal, self.task_execution_complete_cb) 
            
            wiggle_room = rospy.Duration(5)
            # start a timer to kill off tasks that overrun
            self.action_timeout_timer = rospy.Timer(self.active_task.max_duration + wiggle_room, self.cancel_active_task, oneshot=True)

        except Exception, e:
            rospy.logwarn('Exception in start_task_action: %s', e)
            # do bookkeeping before causing update
            completed = self.active_task
            self.active_task = None
            self.active_task_id = Task.NO_TASK
            self.task_failed(completed)

    def start_task_navigation(self):
        # handle delayed start up
        if self.nav_client == None:
            #self.nav_client = actionlib.SimpleActionClient('topological_navigation', GotoNodeAction)
            self.nav_client = actionlib.SimpleActionClient('mdp_plan_exec/execute_policy', ExecutePolicyAction)
            self.nav_client.wait_for_server()
            rospy.logdebug("Created action client")

        # start a timer to kill off tasks that overrun
        self.nav_timeout_timer = rospy.Timer(self.expected_navigation_duration(self.active_task), self.cancel_navigation, oneshot=True)

        #nav_goal = GotoNodeGoal(target = self.active_task.start_node_id)
        nav_goal = ExecutePolicyGoal(task_type=ExecutePolicyGoal.GOTO_WAYPOINT, target_id = self.active_task.start_node_id, time_of_day='all_day')
        self.log_task_event(self.active_task, TaskEvent.NAVIGATION_STARTED, rospy.get_rostime())

        self.nav_client.send_goal(nav_goal, self.navigation_complete_cb)
        rospy.loginfo("navigating to %s" % nav_goal)



    def navigation_complete_cb(self, goal_status, result):

        # print self.nav_client.get_state()

        now = rospy.get_rostime()
        # stop the countdown
        self.nav_timeout_timer.shutdown()

        if self.nav_client.get_state() == GoalStatus.SUCCEEDED:


            rospy.loginfo('Navigation to %s succeeded' % self.active_task.start_node_id)        
            self.log_task_event(self.active_task, TaskEvent.NAVIGATION_SUCCEEDED, now)                

            if self.active_task.action != '':                                        
                self.start_task_action()
            else:
                # do bookkeeping before causing update
                completed = self.active_task
                self.active_task = None
                self.active_task_id = Task.NO_TASK

                self.task_succeeded(completed)
        else:
            if self.nav_client.get_state() == GoalStatus.PREEMPTED:
                rospy.loginfo('Navigation to %s timed out' % self.active_task.start_node_id)        
                self.log_task_event(self.active_task, TaskEvent.NAVIGATION_PREEMPTED, now)                
            else: 
                rospy.loginfo('Navigation to %s failed' % self.active_task.start_node_id)        
                self.log_task_event(self.active_task, TaskEvent.NAVIGATION_FAILED, now)                
        
            # do bookkeeping before causing update
            completed = self.active_task
            self.active_task = None
            self.active_task_id = Task.NO_TASK

            self.task_failed(completed)
            self.log_task_event(completed, TaskEvent.TASK_FINISHED, now)

            


    def task_execution_complete_cb(self, goal_status, result):

        self.action_timeout_timer.shutdown()
        now = rospy.get_rostime()


        # do bookkeeping before causing update
        completed = self.active_task

        self.active_task = None
        self.active_task_id = Task.NO_TASK


        if self.action_client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo('Execution of task %s succeeded' % completed.task_id)        
            self.log_task_event(completed, TaskEvent.EXECUTION_SUCCEEDED, now)                
            self.task_succeeded(completed)
        else:
            if self.action_client.get_state() == GoalStatus.PREEMPTED:
                self.log_task_event(completed, TaskEvent.EXECUTION_PREEMPTED, now)                
            else:
                self.log_task_event(completed, TaskEvent.EXECUTION_FAILED, now)                            
            self.task_failed(completed)

        self.log_task_event(completed, TaskEvent.TASK_FINISHED, now)   


    def log_task_events(self, tasks, event, time, description=""):
        for task in tasks:
            te = TaskEvent(task=task, event=event, time=time, description=description)
            self.logging_msg_store.insert(te)


    def log_task_event(self, task, event, time, description=""):
        te = TaskEvent(task=task, event=event, time=time, description=description)
        self.logging_msg_store.insert(te)


    def add_task_ros_srv(self, req):
        """
        Adds a task into the task execution framework.
        """
        req.task.task_id = self.task_counter
        self.task_counter += 1
        self.add_tasks([req.task])
        self.log_task_event(req.task, TaskEvent.ADDED, rospy.get_rostime())                
        return req.task.task_id
    add_task_ros_srv.type=AddTask


    def add_tasks_ros_srv(self, req):
        """
        Adds a task into the task execution framework.
        """
        task_ids = []
        for task in req.tasks:
            task.task_id = self.task_counter
            task_ids.append(task.task_id)
            self.task_counter += 1

        self.add_tasks(req.tasks)        
        self.log_task_events(req.tasks, TaskEvent.ADDED, rospy.get_rostime())                
        return [task_ids]
    add_tasks_ros_srv.type=AddTasks

    def demand_task_ros_srv(self, req):
        """
        Demand a the task from the execution framework.
        """
        req.task.task_id = self.task_counter        
        self.task_counter += 1
        req.task.execution_time = rospy.get_rostime()

        # stop anything else
        if self.active_task:
            self.cancel_active_task(None)

        # and inform implementation to let it take action
        self.task_demanded(req.task, self.active_task)                        

        self.log_task_event(req.task, TaskEvent.DEMANDED, rospy.get_rostime())                


        return req.task.task_id
    demand_task_ros_srv.type=DemandTask

    def cancel_task_ros_srv(self, req):
        """ Cancel the speficially requested task """        
        if self.active_task is not None and self.active_task.task_id == req.task_id:        
            self.log_task_event(self.active_task, TaskEvent.CANCELLED_MANUALLY, rospy.get_rostime())   
            self.cancel_active_task(None)
            return True
        else:
            self.log_task_event(Task(task_id=req.task_id), TaskEvent.CANCELLED_MANUALLY, rospy.get_rostime())   
            return self.cancel_task(req.task_id)
    cancel_task_ros_srv.type = CancelTask

    def clear_schedule_ros_srv(self, req):
        """ Remove all scheduled tasks and active task """
        if self.active_task is not None:        
            self.cancel_active_task(None)

        self.clear_schedule()
    clear_schedule_ros_srv.type = Empty

    def get_execution_status_ros_srv(self, req):
        return self.executing
    get_execution_status_ros_srv.type = GetExecutionStatus

    def set_execution_status_ros_srv(self, req):
        if self.executing and not req.status:
            rospy.logdebug("Pausing execution")
            self.pause_execution()
        elif not self.executing and req.status:
            rospy.logdebug("Starting execution")
            self.start_execution()
        previous = self.executing
        self.executing = req.status
        return previous
    set_execution_status_ros_srv.type = SetExecutionStatus


    def instantiate_from_string_pair(self, string_pair):
        if len(string_pair.first) == 0:
            return string_pair.second
        elif string_pair.first == Task.INT_TYPE:
            return int(string_pair.second)
        elif string_pair.first == Task.FLOAT_TYPE:
            return float(string_pair.second)     
        elif string_pair.first == Task.TIME_TYPE:
            return rospy.Time.from_sec(float(string_pair.second))
        elif string_pair.first == Task.DURATION_TYPE:
            return rospy.Duration.from_sec(float(string_pair.second))
        else:
            msg = self.msg_store.query_id(string_pair.second, string_pair.first)[0]
            # print msg
            if msg == None:
                raise RuntimeError("No matching object for id %s of type %s" % (string_pair.second, string_pair.first))
            return msg

    def get_arguments(self, argument_list):
        return map(self.instantiate_from_string_pair, argument_list)



