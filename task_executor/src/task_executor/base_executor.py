#!/usr/bin/env python

import rospy

from strands_executive_msgs.msg import Task, TaskEvent

import mongodb_store.util as dc_util
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from mongodb_store.message_store import MessageStoreProxy

from strands_executive_msgs.srv import *

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


    # "Constants" to determine which nav service to use
    TOPOLOGICAL_NAV=0
    MDP_NAV=1

    def __init__(self):
        self.task_counter = 1
        self.msg_store = MessageStoreProxy() 
        self.logging_msg_store = MessageStoreProxy(collection='task_events') 

        self.nav_service = BaseTaskExecutor.MDP_NAV

        nav_service = rospy.get_param('~nav_service', 'mdp')

        if nav_service.lower().startswith('top'):
            self.nav_service = BaseTaskExecutor.TOPOLOGICAL_NAV
            rospy.loginfo('Using topological navigation')
        else: 
            rospy.loginfo('Using mdp navigation')

        self.expected_time_srv = None

        self.executing = False
        # start with some faked but likely one in case of problems
        self.current_node = 'WayPoint1'
        self.closest_node = 'WayPoint1'
        rospy.Subscriber('/current_node', String, self.update_topological_location)
        rospy.Subscriber('/closest_node', String, self.update_topological_closest_node)
        self.active_task = None
        self.active_task_completes_by = rospy.get_rostime()
        self.logging_lock = threading.Lock()
        self.service_lock = threading.Lock()
        self.expected_time_lock = threading.RLock()

        self.task_event_publisher = rospy.Publisher('task_executor/events', TaskEvent, queue_size=20)



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


    def expected_navigation_duration_now(self, end):
        # if we're going nowhere, return some default
        if end == '':
            return rospy.Duration(10)
        elif self.current_node == 'none':
            return self.get_navigation_duration(start=self.closest_node, end=end)
        else:
            return self.get_navigation_duration(start=self.current_node, end=end)

    def create_expected_time_service(self):
        if self.expected_time_srv is None:
            if self.nav_service == BaseTaskExecutor.TOPOLOGICAL_NAV:
                from strands_navigation_msgs.srv import EstimateTravelTime
                expected_time_srv_name = 'topological_navigation/travel_time_estimator'
                rospy.loginfo('Waiting for %s' % expected_time_srv_name)
                rospy.wait_for_service(expected_time_srv_name)
                rospy.loginfo('... and got %s' % expected_time_srv_name)
                self.expected_time_srv = rospy.ServiceProxy(expected_time_srv_name, EstimateTravelTime)        
            elif self.nav_service == BaseTaskExecutor.MDP_NAV:
                from strands_executive_msgs.srv import GetExpectedTravelTimesToWaypoint
                expected_time_srv_name = 'mdp_plan_exec/get_expected_travel_times_to_waypoint'
                rospy.loginfo('Waiting for %s' % expected_time_srv_name)
                rospy.wait_for_service(expected_time_srv_name)
                rospy.loginfo('... and got %s' % expected_time_srv_name)
                self.expected_time_srv = rospy.ServiceProxy(expected_time_srv_name, GetExpectedTravelTimesToWaypoint)        
            else:
                raise RuntimeError('Unknown nav service: %s'% self.nav_service)


    def get_mdp_vector(self, target, epoch):
        try:            
            # expected_time_lock is reentrant
            self.expected_time_lock.acquire()
            self.create_expected_time_service()
            return self.expected_time_srv(target_waypoint=target, epoch=epoch)
        finally:
            self.expected_time_lock.release()

    def mdp_expected_time(self, start, end, task = None):


        # if task is none, assume immediate execution
        if task is None or task.start_after is None:
            epoch = rospy.get_rostime()             
        # else take the epoch from the earliest execution time
        else:
            epoch = task.start_after

        resp = self.get_mdp_vector(end, epoch)

        return resp.travel_times[resp.source_waypoints.index(start)]


    def expected_time(self, start, end, task = None):
        self.create_expected_time_service()

        if self.nav_service == BaseTaskExecutor.TOPOLOGICAL_NAV:
            return self.expected_time_srv(start=start, target=end).travel_time
        elif self.nav_service == BaseTaskExecutor.MDP_NAV:
            return self.mdp_expected_time(start, end, task)
        else:
            raise RuntimeError('Unknown nav service: %s'% self.nav_service)

    def get_navigation_duration(self, start, end, task = None):

        try:            
            # prevent concurrent calls to expected_time service. 
            self.expected_time_lock.acquire()
            if start == '' or end == '':
                # if we're going nowhere, return some default
                return rospy.Duration(10)
            else:
                et = self.expected_time(start, end, task)
                # rospy.loginfo('expected travel time %s' % et)                
                return rospy.Duration(max(et.to_sec(), 10))
        except Exception, e:
            rospy.logwarn('Caught exception when getting expected time: %s' % e)
            return rospy.Duration(10)
        finally:
            self.expected_time_lock.release()
                    

        
    def log_task_events(self, tasks, event, time, description=""):
        try:
            self.logging_lock.acquire()
            for task in tasks:
                te = TaskEvent(task=task, event=event, time=time, description=description)
                try:
                    self.task_event_publisher.publish(te)
                    self.logging_msg_store.insert(te)
                except Exception, e:
                    rospy.logwarn('Caught exception when logging: %s' % e)
        finally:
            self.logging_lock.release()


    def log_task_event(self, task, event, time, description=""):
        try:
            self.logging_lock.acquire()
            te = TaskEvent(task=task, event=event, time=time, description=description)
            self.task_event_publisher.publish(te)
            self.logging_msg_store.insert(te)
        except Exception, e:
            rospy.logwarn('Caught exception when logging: %s' % e)
        finally:
            self.logging_lock.release()


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

    def get_active_task_ros_srv(self, req):
        """
        Gets the currently executing task.
        """
        return [self.active_task]
    get_active_task_ros_srv.type=GetActiveTask

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
        try:            
            self.service_lock.acquire()

            if self.active_task is not None and not self.is_task_interruptible(self.active_task):
                return [False, 0, self.active_task_completes_by - rospy.get_rostime()]

            req.task.task_id = self.task_counter        
            self.task_counter += 1
            req.task.execution_time = rospy.get_rostime()



            # stop anything else
            if self.active_task is not None:
                self.pause_execution()
                self.executing = False
                self.cancel_active_task()

            # and inform implementation to let it take action
            self.task_demanded(req.task, self.active_task)                        

            if not self.executing:
                self.start_execution()

            self.log_task_event(req.task, TaskEvent.DEMANDED, rospy.get_rostime())                
            return [True, req.task.task_id, rospy.Duration(0)]        
        finally:    
            self.service_lock.release()


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

        success = False

        remaining_time = rospy.Duration(0)


        if self.executing and not req.status:
            rospy.logdebug("Pausing execution")

            previous = self.executing
            

            if self.is_task_interruptible(self.active_task):
                self.pause_execution()
                self.executing = False
                success = True            
            else:
                remaining_time = self.active_task_completes_by - rospy.get_rostime()

        elif not self.executing and req.status:
            rospy.logdebug("Starting execution")

            previous = self.executing
            self.executing = True
        
            self.start_execution()
            success = True
        else:
            previous = self.executing            
            success = True

        self.service_lock.release()

        return [previous, success, remaining_time]
    set_execution_status_ros_srv.type = SetExecutionStatus


    def prepare_task(self, task):

        self.active_task = task       

        now = rospy.get_rostime()

        expected_nav_duration = rospy.Duration(0)
        if self.active_task.start_node_id != '':                    
            expected_nav_duration = self.expected_navigation_duration_now(self.active_task.start_node_id)
            rospy.loginfo('expected_nav_duration:  %s' % expected_nav_duration.to_sec())

        total_task_duration = expected_nav_duration + task.max_duration
        
        self.active_task_completes_by = now + total_task_duration    

        return expected_nav_duration


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


    def is_task_interruptible(self, task):
        if task is None:
            rospy.logwarn('is_task_interruptible passed a None')
            return False

        try:
            srv_name = task.action + '_is_interruptible'
            rospy.wait_for_service(srv_name, timeout=1)    
            is_interruptible = rospy.ServiceProxy(srv_name, IsTaskInterruptible)
            return is_interruptible().status
        except rospy.ROSException as exc:
            rospy.logdebug('%s service does not exist, treating as interruptible')
            return True
        except rospy.ServiceException as exc:
            rospy.logwarn(exc.message)
            return True



    




