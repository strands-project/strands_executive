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
import threading

class BaseTaskExecutor(object):
    # These can be implemented by sub classes to provide hooks into the execution system

    def add_tasks(self, tasks):
        """ Called with new tasks for the executor """
        pass

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
        expected_time_srv_name = '/mdp_plan_exec/get_expected_travel_time_to_node'
        rospy.loginfo('Waiting for %s' % expected_time_srv_name)
        rospy.wait_for_service(expected_time_srv_name)
        rospy.loginfo('... and got %s' % expected_time_srv_name)
        self.expected_time = rospy.ServiceProxy(expected_time_srv_name, GetExpectedTravelTime)
        self.executing = False
        # start with some faked but likely one in case of problems
        self.current_node = 'WayPoint1'
        self.closest_node = 'WayPoint1'
        rospy.Subscriber('/current_node', String, self.update_topological_location)
        rospy.Subscriber('/closest_node', String, self.update_topological_closest_node)
        self.active_task = None
        self.active_task_completes_by = rospy.get_rostime()
        self.service_lock = threading.Lock()


    def get_active_task_completion_time(self):
        return self.active_task_completes_by

    def update_topological_location(self, node_name):
        self.current_node = node_name.data

    
    def update_topological_closest_node(self,node_name):
        self.closest_node=node_name.data


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
        if self.current_node == 'none':
            et = self.expected_time(start_id=self.closest_node, target_id=task.start_node_id,time_of_day="all_day")
        else:
            et = self.expected_time(start_id=self.current_node, target_id=task.start_node_id,time_of_day="all_day")
        # rospy.loginfo('expected travel time %s' % et.travel_time)
        # allow a bit of time for any transition -- mainly for testing cases
        return rospy.Duration(max(et.travel_time * 3, 10))
        # return rospy.Duration(120)


    def log_task_events(self, tasks, event, time, description=""):
        for task in tasks:
            te = TaskEvent(task=task, event=event, time=time, description=description)

            try:
                self.logging_msg_store.insert(te)
            except Exception, e:
                rospy.logwarn('Caught exception when logging: %s' % e)




    def log_task_event(self, task, event, time, description=""):
        te = TaskEvent(task=task, event=event, time=time, description=description)
        try:
            self.logging_msg_store.insert(te)
        except Exception, e:
            rospy.logwarn('Caught exception when logging: %s' % e)


    def add_task_ros_srv(self, req):
        """
        Adds a task into the task execution framework.
        """
        self.service_lock.acquire()
        req.task.task_id = self.task_counter
        self.task_counter += 1
        self.add_tasks([req.task])
        self.log_task_event(req.task, TaskEvent.ADDED, rospy.get_rostime())                
        self.service_lock.release()
        return req.task.task_id
    add_task_ros_srv.type=AddTask


    def add_tasks_ros_srv(self, req):
        """
        Adds a task into the task execution framework.
        """
        self.service_lock.acquire()
        task_ids = []
        for task in req.tasks:
            task.task_id = self.task_counter
            task_ids.append(task.task_id)
            self.task_counter += 1

        self.add_tasks(req.tasks)        
        self.log_task_events(req.tasks, TaskEvent.ADDED, rospy.get_rostime())                
        self.service_lock.release()
        return [task_ids]
    add_tasks_ros_srv.type=AddTasks

    def demand_task_ros_srv(self, req):
        """
        Demand a the task from the execution framework.
        """
        self.service_lock.acquire()
        req.task.task_id = self.task_counter        
        self.task_counter += 1
        req.task.execution_time = rospy.get_rostime()

        # stop anything else
        if self.active_task is not None:
            self.cancel_active_task()

        # and inform implementation to let it take action
        self.task_demanded(req.task, self.active_task)                        

        self.log_task_event(req.task, TaskEvent.DEMANDED, rospy.get_rostime())                

        self.service_lock.release()
        return req.task.task_id
    demand_task_ros_srv.type=DemandTask

    def cancel_task_ros_srv(self, req):
        """ Cancel the speficially requested task """        

        self.service_lock.acquire()
        
        cancelled = False
        if self.active_task is not None and self.active_task.task_id == req.task_id:        
            self.log_task_event(self.active_task, TaskEvent.CANCELLED_MANUALLY, rospy.get_rostime())   
            self.cancel_active_task()
            cancelled = True
        else:
            self.log_task_event(Task(task_id=req.task_id), TaskEvent.CANCELLED_MANUALLY, rospy.get_rostime())   
            cancelled = self.cancel_task(req.task_id)
        
        self.service_lock.release() 

        return cancelled
    cancel_task_ros_srv.type = CancelTask

    def clear_schedule_ros_srv(self, req):
        """ Remove all scheduled tasks and active task """
        
        self.service_lock.acquire()        

        self.clear_schedule()

        if self.active_task is not None:        
            self.cancel_active_task()

        self.service_lock.release()

        return EmptyResponse()

    clear_schedule_ros_srv.type = Empty

    def get_execution_status_ros_srv(self, req):
        return self.executing
    get_execution_status_ros_srv.type = GetExecutionStatus

    def set_execution_status_ros_srv(self, req):
        
        self.service_lock.acquire()

        if self.executing and not req.status:
            rospy.logdebug("Pausing execution")
            self.pause_execution()
        elif not self.executing and req.status:
            rospy.logdebug("Starting execution")
            self.start_execution()
        previous = self.executing
        self.executing = req.status
        
        self.service_lock.release()

        return previous
    set_execution_status_ros_srv.type = SetExecutionStatus


    def prepare_task(self, task):

        self.active_task = task       

        now = rospy.get_rostime()

        expected_nav_duration = rospy.Duration(0)
        if self.active_task.start_node_id != '':                    
            expected_nav_duration = self.expected_navigation_duration(task)
            rospy.loginfo('expected_nav_duration:  %s' % expected_nav_duration)

        total_task_duration = expected_nav_duration + task.max_duration
        
        self.active_task_completes_by = now + total_task_duration    

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
        elif string_pair.first == Task.BOOL_TYPE:   
            return string_pair.second == 'True'
        else:
            msg = self.msg_store.query_id(string_pair.second, string_pair.first)[0]
            # print msg
            if msg == None:
                raise RuntimeError("No matching object for id %s of type %s" % (string_pair.second, string_pair.first))
            return msg

    def get_arguments(self, argument_list):
        return map(self.instantiate_from_string_pair, argument_list)



class AbstractTaskExecutor(BaseTaskExecutor):

    def __init__(self):
        super( AbstractTaskExecutor, self ).__init__()      
               
        self.nav_client = None
        self.action_client = None
        
    def execute_task(self, task):

        self.log_task_event(task, TaskEvent.TASK_STARTED, rospy.get_rostime())


        self.prepare_task(task)

        if self.active_task.start_node_id != '':                    
            self.start_task_navigation(expected_nav_duration)
        elif self.active_task.action != '':                    
            self.start_task_action()
        else:
            warning = 'Provided task had no start_node_id or action %s' % self.active_task
            rospy.logwarn(warning)
            self.log_task_event(self.active_task, TaskEvent.TASK_COMPLETE, now, warning)

            self.active_task = None            

    def cancel_active_task(self):
        self.cancel_active_task_cb(None)

    def cancel_active_task_cb(self, event):
        """ Cancel any active nav or task action """
        if event is not None:
            rospy.logwarn("Cancelling task that has overrun %s" % self.active_task.task_id)
        else:
            rospy.logwarn("Cancelling task %s" % self.active_task.task_id)

        now = rospy.get_rostime()
        cancelled = self.active_task
        self.active_task = None

        if self.nav_client is not None: 
            self.nav_timeout_timer.shutdown()
            if self.nav_client.get_state() == GoalStatus.ACTIVE:
                self.nav_client.cancel_goal()                            
            self.nav_client = None
            self.log_task_event(cancelled, TaskEvent.NAVIGATION_PREEMPTED, now)                

            
        if self.action_client  is not None:
            self.action_timeout_timer.shutdown()
            if self.action_client.get_state() == GoalStatus.ACTIVE:
                self.action_client.cancel_goal()            
            self.action_client = None
            self.log_task_event(cancelled, TaskEvent.EXECUTION_PREEMPTED, now)                        


        self.task_failed(cancelled)
        
        if event is not None:
            self.log_task_event(cancelled, TaskEvent.CANCELLED_MANUALLY, now)
        
        self.log_task_event(cancelled, TaskEvent.TASK_FINISHED, now)

    def cancel_navigation(self, event):
        """ Called on nav timeout """
        rospy.logwarn("Cancelling navigation that has overrun for task %s" % self.active_task.task_id)

        # cancel navigation
        if self.nav_client is not None:
            if self.nav_client.get_state() == GoalStatus.ACTIVE:
                self.nav_client.cancel_goal()
            self.nav_client = None

            # fail task 
            now = rospy.get_rostime()
            cancelled = self.active_task
            self.active_task = None

            self.log_task_event(cancelled, TaskEvent.NAVIGATION_PREEMPTED, now)                
            self.task_failed(cancelled)
            self.log_task_event(cancelled, TaskEvent.TASK_FINISHED, now)
        else:
            rospy.logwarn('Tried to cancel a navigation action that was None')

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
            self.action_timeout_timer = rospy.Timer(self.active_task.max_duration + wiggle_room, self.cancel_active_task_cb, oneshot=True)

        except Exception, e:
            rospy.logwarn('Exception in start_task_action: %s', e)
            # do bookkeeping before causing update
            completed = self.active_task
            self.active_task = None            
            self.task_failed(completed)

    def start_task_navigation(self, expected_duration):
        # handle delayed start up
        # if self.nav_client == None:
        # always reconnect in case of issues before
        self.nav_client = actionlib.SimpleActionClient('mdp_plan_exec/execute_policy', ExecutePolicyAction)
        self.nav_client.wait_for_server()
        rospy.logdebug("Created action client")

        # start a timer to kill off tasks that overrun
        self.nav_timeout_timer = rospy.Timer(expected_duration, self.cancel_navigation, oneshot=True)
        # self.nav_timeout_timer = rospy.Timer(rospy.Duration(360), self.cancel_navigation, oneshot=True)

        #nav_goal = GotoNodeGoal(target = self.active_task.start_node_id)
        nav_goal = ExecutePolicyGoal(task_type=ExecutePolicyGoal.GOTO_WAYPOINT, target_id = self.active_task.start_node_id, time_of_day='all_day')
        self.log_task_event(self.active_task, TaskEvent.NAVIGATION_STARTED, rospy.get_rostime())

        self.nav_client.send_goal(nav_goal, self.navigation_complete_cb)
        rospy.loginfo("navigating to %s for action %s" % (self.active_task.start_node_id, self.active_task.action))



    def navigation_complete_cb(self, goal_status, result):

        # if self.nav_client was set to None then navigation was cancelled and we don't care about the response to the call back

        if self.nav_client is not None:

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

                self.task_failed(completed)
                self.log_task_event(completed, TaskEvent.TASK_FINISHED, now)

            


    def task_execution_complete_cb(self, goal_status, result):

        if self.action_client is not None:
            self.action_timeout_timer.shutdown()
            now = rospy.get_rostime()


            # do bookkeeping before causing update
            completed = self.active_task

            self.active_task = None            

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


    




