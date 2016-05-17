#!/usr/bin/env python

import rospy
from Queue import Queue, Empty
from strands_executive_msgs.msg import Task, ExecutionStatus, DurationMatrix, DurationList
from strands_executive_msgs.srv import GetSchedule
from task_executor.sm_base_executor import AbstractTaskExecutor
from task_executor.base_executor import BaseTaskExecutor
from threading import Thread
from task_executor.execution_schedule import ExecutionSchedule
from operator import attrgetter
import copy
from math import floor


# def task_list_diff(all_tasks, subset_of_all_tasks):
#     """
#     Returns all_tasks - subset_of_all_tasks based on task ids
#     """
#     difference = []
#     for at in all_tasks:
#         if not any(st for st in subset_of_all_tasks if st.task_id == at.task_id):
#             difference.append(at)
#     return difference


class ScheduledTaskExecutor(AbstractTaskExecutor):

    def __init__(self):
        # init node first, must be done before call to super init for service advertising to work
        rospy.init_node("task_executor", log_level=rospy.INFO)

        # init superclasses
        super( ScheduledTaskExecutor, self ).__init__()

        # service for scheduler
        schedule_srv_name = 'get_schedule'
        rospy.logdebug('Waiting for %s service' % schedule_srv_name)
        rospy.wait_for_service(schedule_srv_name)
        self.schedule_srv = rospy.ServiceProxy(schedule_srv_name, GetSchedule)

        # topic on which current schedule is broadcast
        self.schedule_publisher = rospy.Publisher('/current_schedule', ExecutionStatus, queue_size=1)

    #Lenka why this is here?
        # defaults for setting the ends of tasks
        self.default_duration = rospy.Duration.from_sec(60 * 60 * 4)
        
        # storage for tasks which have been added but not considered 
        self.unscheduled_tasks = Queue()
       
        # data structure that manages tasks
        self.execution_schedule = ExecutionSchedule(travel_duration_fn=self.expected_navigation_duration_now)

       
        self.running = False

        self.advertise_services()




    def start_execution(self):
        """ Called when overall execution should  (re)start """
        
        if not self.running:
            self.running = True

            self.scheduling_thread = Thread(target=self.schedule_tasks)    
            self.execution_thread = Thread(target=self.execute_tasks)

            self.scheduling_thread.start()    
            self.execution_thread.start()


    def pause_execution(self):
        
        if self.running:
            rospy.loginfo('Pausing execution')
            self.running = False

            # save the tasks we had previously
            previously_scheduled = self.execution_schedule.get_schedulable_tasks()
            current_task = self.execution_schedule.get_current_task()

            # clear all tasks that were previously scheduled
            self.execution_schedule.clear_schedule()
            
            if current_task is not None:
                rospy.loginfo('Interrupting active task')

                # stop execution
                self.cancel_active_task()            

                # wait for current task to end
                self.wait_for_task_to_complete()
            
                # add the interrupted task back to the queue for scheduling later
                self.unscheduled_tasks.put(current_task)

            for task in previously_scheduled:
                self.unscheduled_tasks.put(task)                


       

    def get_default_end_time(self, start_time):
        return start_time + self.default_duration


    def fill_times(self, task):
        if task.start_after.is_zero():            
            task.start_after = rospy.get_rostime()

        if task.end_before.is_zero():
            task.end_before = self.get_default_end_time(task.start_after)


    def add_tasks(self, tasks):
        """ Called with new tasks for the executor """
        
        for task in tasks:
            self.fill_times(task)
            # if no end node, set it to the same as the start
            # note that if start is blank, this isn't going to help later
            if task.end_node_id == '':
                task.end_node_id = task.start_node_id 

        for task in tasks:
            self.unscheduled_tasks.put(task)

    def task_complete(self, task):
        """ Called when the given task has completed execution """
        # pass signal to schedule
        self.execution_schedule.task_complete(task) 

    def task_succeeded(self, task):
        """ Called when the given task has completed execution successfully """
        self.task_complete(task)

    def task_failed(self, task):
        """ Called when the given task has completed execution but failed """
        self.task_complete(task)


    def wait_for_task_to_complete(self):
        """ Useful for cases where you don't know what the effect of cancellation will be   """

        wait_count = 0
        # a similar theshold is used in sm_base_executor to wait for termination of active task
        wait_threshold = 31
        while self.execution_schedule.get_current_task() is not None and wait_count < wait_threshold and not rospy.is_shutdown():
            rospy.sleep(1)
            rospy.loginfo('Waiting for previous task to finish')
            wait_count += 1

        # if it's still not None then we got bored of waiting
        if self.execution_schedule.get_current_task() is not None:
            rospy.logwarn('Previous task did not terminate nicely. Everything from here on in could be dicey.')
            self.execution_schedule.current_task = None


    def task_demanded(self, demanded_task, currently_active_tasks):
        """ Called when a task is demanded. self.active_tasks contains the demanded task (and is being executed) and previously_active_task was the task that was being executed (which could be None) """

        if demanded_task.end_node_id == '':
            demanded_task.end_node_id = demanded_task.start_node_id 

        rospy.loginfo('Task %s, %s demanded from scheduler' % (demanded_task.task_id, demanded_task.action))

        # save the tasks we had previously
        previously_scheduled = self.execution_schedule.get_schedulable_tasks()

        
        # clear all tasks that were previously scheduled
        self.execution_schedule.clear_schedule()

        # wait until the demanded task has been taken off for execution, otherwise we run into problems with get_schedulable_tasks
        # cancellation should block until cancelled 
        self.wait_for_task_to_complete()

        # try to schedule them back in 
        self.execution_schedule.add_new_tasks([demanded_task])
        # put scheduled tasks back into execution. this will trigger a change in execution if necessary
        self.execution_schedule.set_schedule([demanded_task])


        # now try to put the other tasks back in
        if len(previously_scheduled) > 0:
            success, added = self.try_schedule(previously_scheduled)
            if success:
                rospy.loginfo('Was able to reinstate %s/%s tasks after demand' % (len(added), len(previously_scheduled)))
                if len(currently_active_tasks) > 0:
                    success, added = self.try_schedule(currently_active_tasks)
                    if success and len(added) > 0:
                        rospy.loginfo('Was also able to reinstate previously active task after demand')
                    else:
                        self.drop_tasks([currently_active_task], description = "Dropped because it couldn't be reinstated after a demanded task")
                        rospy.loginfo('Was not able to reinstate previously active task after demand (but other tasks ok)')
            else:
                self.drop_tasks(previously_scheduled, description = "Dropped because it couldn't be reinstated after a demanded task")
                rospy.loginfo('Was NOT able to reinstate tasks after demand')


    def get_duration_matrix(self, tasks):
        """
        Creates the matrix of durations between waypoints needed as input to the scheuler.
        Output is a DurationMatrix encoding  duration[i][j] where this is the duration expected for travelling between the end of the ith task in tasks and the start of the jth element.
        """
        if self.nav_service == BaseTaskExecutor.TOPOLOGICAL_NAV:
            return self.get_duration_matrix_top_nav(tasks)
        elif self.nav_service == BaseTaskExecutor.MDP_NAV:
            return self.get_duration_matrix_mdp(tasks)
        else:
            raise RuntimeError('Unknown nav service: %s'% self.nav_service)

    def get_duration_matrix_mdp(self, tasks):
        """
        Creates the matrix of durations between waypoints needed as input to the scheuler.
        Output is a DurationMatrix encoding  duration[i][j] where this is the duration expected for travelling between the end of the ith task in tasks and the start of the jth element.
        """

        # mdp returns vector of times from ALL START nodes to a SINGLE TARGET for a SINGLE TIME 
        # therefore we can combine calls from any start node to the same target for the same time 


        # used to cache reseults of calls to mdp service
        travel_durations = dict()

        # thing we need to return
        dm = DurationMatrix()

        for first_task in tasks:
            dm.durations.append(DurationList())
            for second_task in tasks:

                start = first_task.end_node_id
                epoch = second_task.start_after
                target = second_task.start_node_id

                # if the task should happen where the robot is currently stood then the travel is free
                if target == '':
                    dm.durations[-1].durations.append(rospy.Duration(0))
                else:
                    # we need to get the travel_duration for this tuple if we haven't seen the time before or the target for this time before
                    if epoch not in travel_durations or target not in travel_durations[epoch]:
                        if epoch not in travel_durations:
                            travel_durations[epoch] = dict()
                        # call mdp duration service
                        resp = self.get_mdp_vector(target, epoch)
                        travel_durations[epoch][target] = resp
                    else:
                        # rospy.loginfo('Saving a call: %s %s %s', start, epoch.secs, target)                    
                        resp = travel_durations[epoch][target]

                    dm.durations[-1].durations.append(resp.travel_times[resp.source_waypoints.index(start)])

        return dm

    def get_duration_matrix_top_nav(self, tasks):
        """
        Creates the matrix of durations between waypoints needed as input to the scheuler.
        Output is a DurationMatrix encoding  duration[i][j] where this is the duration expected for travelling between the end of the ith task in tasks and the start of the jth element.
        """
        # first populate sets of start and end locations for moves between tasks, these are the end and start of tasks repsectively
        start_nodes = set([task.end_node_id for task in tasks])
        end_nodes = set([task.start_node_id for task in tasks])

        # next get the costs for each element of the cross product
        durations = dict()
        for start in start_nodes:
            durations[start] = dict()
            for end in end_nodes:
                durations[start][end] = self.get_navigation_duration(start, end)


        # now populate the DurationMatrix object
        dm = DurationMatrix()
        for task_i in tasks:
            dm.durations.append(DurationList())
            for task_j in tasks:
                dm.durations[-1].durations.append(durations[task_i.end_node_id][task_j.start_node_id])

        return dm




    def call_scheduler(self, tasks, earliest_start, current_id=0):
        """ 
        
        Calls scheduler. Reorders the list of tasks in execution order with their execution times set. 
        
        """
        resp = self.schedule_srv(tasks, earliest_start, current_id, self.get_duration_matrix(tasks)) # Lenka note: why does the service need earliest start and current id?

        # rospy.loginfo(resp)

        if len(resp.task_order) > 0:
            

            # add start times to a dictionary for fast lookup
            task_times = {}
            for (task_id, start_time) in zip(resp.task_order, resp.execution_times):
                # print task_id, start_time
                task_times[task_id] = start_time

            # set start times inside of tasks
            for task in tasks:
                # add min_window back on to starting times
                # print task.task_id, task_times[task.task_id] 
                task.execution_time = task_times[task.task_id] 

                # taking out as rescheduling demanded tasks have an issue here I think
                # assert task.execution_time >= task.start_after
                # assert task.execution_time + task.max_duration <= task.end_before

                # print 'task %s will start at %s.%s' % (task.task_id, task.execution_time.secs, task.execution_time.nsecs)                        

            tasks.sort(key=attrgetter('execution_time')) # Lenka note: is "tasks" return to someone? otherwise sorting makes no affect
            return True
        
        else:
            rospy.loginfo('No schedule found')
            return False


    def publish_schedule(self):
        # pub.publish(std_msgs.msg.String("foo"))
        
        exe_status = ExecutionStatus()
        exe_status.header.stamp = rospy.get_rostime()

        current = self.execution_schedule.get_current_task()
        
        if current:
            exe_status.currently_executing = True              
            exe_status.execution_queue.append(current)

        exe_status.execution_queue.extend(self.execution_schedule.get_execution_queue())
        # print schedule
        self.schedule_publisher.publish(exe_status)

    def bound_tasks_by_start_window(self, tasks, active_task):

      bounded_tasks = []
      dropped_tasks = []

      # this is the lower bound on future execution time
      start_after = rospy.get_rostime()

      if active_task is not None:
          start_after = self.get_active_task_completion_time()
          for task in tasks:
            # if it's still executable after the bound
            if start_after + task.max_duration <= task.end_before:
                bounded_tasks.append(task)                            
            else:
                if(active_task.priority >= task.priority):
                  dropped_tasks.append(task)
                else: #if task have higher priority, we want to propagate it, it might be scheduled as on demand task later in code
                  bounded_tasks.append(task)      
      else: #there is no active task, thus we just check if task can be done according now() time        
        for task in tasks:
            # if it's still executable after the bound
            if start_after + task.max_duration <= task.end_before:
                # 
                # This is no longer done because it means that execution also has to work with a bounded start_after which may actually lead to unnecessary delays
                # 
                # # if we need to push it back to the bound
                # if start_after > task.start_after:
                    # # rospy.loginfo('pushing boundary')
                    # task.start_after = start_after
                bounded_tasks.append(task)                            
            else:
                dropped_tasks.append(task)                            

      return bounded_tasks, dropped_tasks, start_after

    def throw_away_tasks(self, all_tasks, amount):
        '''This method throw away tasks based on priorities, or if tasks have same priority, than amount parameter is applied.
           Returns true if tasks were throwen away (have lower prio than active_priority), ortherwise return False'''

        
        amount_tasks = len(all_tasks)
        
        if(amount_tasks == 0):
          rospy.info('EXECUTOR: Throwing of tasks called but there is nothing to throw away.')
          return [], [], False

        all_tasks.sort(key=attrgetter('priority'),reverse=True) # highest priority first
        low_prio = all_tasks[amount_tasks -1] # get the lowest prio
        index = all_tasks.index(low_prio) # find first occurencce of low_prio
       
        
        low_prio_amount = amount_tasks -index #get how many tasks have the lowest prio

        throw_num = floor(amount*amount_tasks ) # compute amount percentage


        if (throw_num < 1):  #throw always at least one
          throw_num = 1

            
        throwen_away = []
        sub_additional = [] # tasks what will be added

        throw_num =  min([low_prio_amount, throw_num])# choose smaller amount of tasks to throw away 
  
        throw_index =amount_tasks - throw_num 


        if len(self.active_tasks) > 0:
          # for the scheduled_task_execution active_tasks will contain at most 1 task
          if(low_prio.priority<= self.active_tasks[0].priority): #we want to throw away tasks
            rospy.loginfo('Schedule not found, trying to discard %s tasks with priority %s', str(throw_num), str(low_prio.priority))
            #if there are more low priority task, only amount% will be throwen away...

            for i in range(0,amount_tasks):
              if i < throw_index:
                sub_additional.append(all_tasks[i])
              else:
                throwen_away.append(all_tasks[i])

            return sub_additional, throwen_away, True
          else: #task which should be throwen away has higher priority then active task
            if len(self.active_tasks) > 0 and not self.is_task_interruptible(self.active_tasks[0]): #we cant preemt, thus returninf True and throwing away task
              rospy.loginfo('Active task cannot be preempted.')
              rospy.loginfo('Schedule not found, trying to discard %s tasks with priority higher prio %s', str(throw_num), str(low_prio.priority))
              
              for i in range(0,amount_tasks):
                if i < throw_index:
                  sub_additional.append(all_tasks[i])
                else:
                  throwen_away.append(all_tasks[i])

              return sub_additional, throwen_away, True
            else: #task is interruptible, we want to preempt -> return False
              return all_tasks, [], False
        else: #nothing running yet
          rospy.loginfo('Schedule not found, trying to discard %s tasks with priority %s', str(throw_num), str(low_prio.priority))
            #if there are more low priority task, only amount% will be throwen away...

          for i in range(0,amount_tasks):
            if i < throw_index:
              sub_additional.append(all_tasks[i])
            else:
              throwen_away.append(all_tasks[i])

         
          return sub_additional, throwen_away, True  

    def try_schedule(self, additional_tasks):
      '''Trying to call scheduler to find a schedule. Also, some tasks might be dropped here.'''

      # bound additional tasks by this bound
      if(len(additional_tasks)>0):
        bounded_tasks, dropped_tasks, lower_bound = self.bound_tasks_by_start_window(additional_tasks, self.execution_schedule.get_current_task())
        
        if len(bounded_tasks) < len(additional_tasks):
          rospy.logwarn('Dropped %s additional tasks which are no longer executable' % (len(additional_tasks) - len(bounded_tasks)))
          self.drop_tasks(dropped_tasks, description = "Dropped because it is now not possible to be executed in the time available")
        additional_tasks = bounded_tasks
      
    

      # these are previoulsy scheduled tasks which we must now reschedule
      schedulable_tasks = self.execution_schedule.get_schedulable_tasks()
      
      if(len(schedulable_tasks)>0):         
        bounded_tasks, dropped_tasks, lower_bound = self.bound_tasks_by_start_window(schedulable_tasks, self.execution_schedule.get_current_task())
        
        if len(bounded_tasks) < len(schedulable_tasks):
          rospy.logwarn('Dropped %s existing tasks which are no longer executable' % (len(schedulable_tasks) - len(bounded_tasks)))
          self.drop_tasks(dropped_tasks, description = "Dropped because it is now not possible to be executed in the time available")
          # have to remove these from schedule too, although this assumes successful scheduling
          # TODO: what if scheduling is not successful?
          self.execution_schedule.remove_tasks(dropped_tasks)
        schedulable_tasks = bounded_tasks
     
     
     
      # the tasks to try and schedule
      to_schedule = []
      # add in the schedulable tasks we already have
      to_schedule.extend(schedulable_tasks)
        
      to_old_schedule = []
      to_old_schedule.extend(to_schedule)           

      #Lenka note: this part makes no sense, why the existing is added to schedule? -> commented out
      ## if we are currently executing something, including this for the start of the schedule too
      #current_id = 0
      #current_task = copy.deepcopy(self.execution_schedule.get_current_task())
      #if current_task is not None:
      #    current_id = current_task.task_id
      #    rospy.loginfo('Including current task %s in scheduling problem' % current_id)
      #    # assuming this executed when requested (may not be!) update how much time is left
      #    current_task.max_duration = rospy.Time(1)
      #    current_task.end_before = lower_bound
      #    current_task.start_after = lower_bound - rospy.Time(2)
      #    to_schedule.append(current_task)

      # add in the ones we have just been given
      to_schedule.extend(additional_tasks)

      current_id = 0 #TODO lenka delete this from scheduler service?

      throwen_away = []
      
      if(len(to_schedule)>0): #only if I have something to schedule :)
        if(len(additional_tasks)==0): #method was called only to reschedule old ones
            if self.call_scheduler(to_schedule, lower_bound, current_id):
                # if this was successful, add new tasks into execution schedule
                #self.execution_schedule.add_new_tasks(additional_tasks)

                #Lenka note: why the actual task is removed? Makes no sense with adding -> commented out
                # and remove the current task from the new schedule
                #to_schedule = [t for t in to_schedule if t.task_id != current_id]

                # put scheduled tasks back into execution. this will trigger a change in execution if necessary
                self.execution_schedule.set_schedule(to_schedule)

                rospy.loginfo('Only rescheduling old schedule, total tasks to perform %s' % self.execution_schedule.get_execution_queue_length())
                return True, additional_tasks
            else:
                # don't need to drop here as additional_tasks is the empy list!
                return False, []

        else: #we have new tasks
            sched_result = self.call_scheduler(to_schedule, lower_bound, current_id)
            all_throwen = [] # to save all throwen tasks
        
            while(not sched_result and (len(additional_tasks)>0)): #schedule is not found, but we have still tasks to throw away
                
                sub_additional, throwen_away, priority_reached = self.throw_away_tasks(to_schedule, 0.2) #throw away 20% if tasks have same prio

                for drop in throwen_away:
                  all_throwen.append(drop)             

                # maybe it will be nice to add last step more softer - like throwing tasks one by one 

                
         
                
                if(not priority_reached): #there are only tasks with higher priority than active task has
                                    
                  self.pause_execution() #this will preempt active task and put it to unscheduled task with previously scheduled tasks as well
                  #convert queue to the list
                  list_unsched = []
                  # now check for any remaining tasks in the queue
                  try:
                    while True:
                        list_unsched.append(self.unscheduled_tasks.get(False))
                  except Empty, e:
                    pass
                  
                  #remove all tasks which were previously scheduled
                  #old_tasks = self.execution_schedule.get_schedulable_tasks()
                  #for taskO in old_tasks:
                  #  self.execution_schedule.remove_task_with_id(taskO.task_id)
                  self.execution_schedule.clear_schedule()
                  
                  unF = 0
                  for taskN in sub_additional: #put all tasks to unscheduled
                    for taskU in list_unsched:
                      if taskN.task_id == taskU.task_id:
                        unF = unF +1 
                    if unF == 0: #the taskN is not in unscheduled
                      taskN.execution_time = rospy.Time(0) # for beeing sure, set execution to zeto
                      list_unsched.append(copy.deepcopy(taskN)) # create a new instance
                      unF = 0
                    else: #the task is already in unschedule, do nothing
                      unF = 0  
                  
                  for taskU in list_unsched: #put all unscheduled tasks to the queue
                    self.unscheduled_tasks.put(taskU)

                  self.start_execution() #start again from scratch, TODO: this will create new threads? how to kill previous once?
                  return False, []

                else: #we throw something away
                  to_schedule = []
                  #to_schedule.extend(to_old_schedule)
                  to_schedule.extend(sub_additional)
                  additional_tasks = sub_additional #rewrite original set of added tasks
                  sched_result = self.call_scheduler(to_schedule, lower_bound, current_id) 
  

            if(len(additional_tasks) == 0):
              rospy.loginfo('EXECUTOR: All tasks (new and not executed) were throwen away') 
              old_tasks = self.execution_schedule.get_schedulable_tasks()
              for taskD in all_throwen:
                for taskO in old_tasks:
                  if taskD.task_id == taskO.task_id: #droped task is in old task, we need to remove it
                    self.execution_schedule.remove_task_with_id(taskO.task_id)
                       

              if(not sched_result): #scheduler didnt find a solution by while loop ended because there are no additional tasks   
                #rather then setting empty schedule, leave there previous schedule
                #self.execution_schedule.set_schedule([])       
                return False, []
              else: #scheduler was successfull, but all tasks were throwen away, to_schedule is empty
                #rather then setting empty schedule, leave there previous schedule              
                #self.execution_schedule.set_schedule(to_schedule)
                rospy.loginfo('EXECUTOR: all tasks were throwen away, scheduler found a primitive (empty) solution')
                #rospy.loginfo('Added %s tasks into the schedule to get total of %s' % (len(additional_tasks), self.execution_schedule.get_execution_queue_length()))
                return True, []



            else: #scheduler was successfull, and we have new tasks to add
            
                #self.execution_schedule.add_new_tasks(additional_tasks)
                #we need to add new tasks, but meanwhile we lost track whihc are new (in drop method)
                old_tasks = self.execution_schedule.get_schedulable_tasks()
                new_tasks = []
                newF = 0
                
                if(len(old_tasks)>0):
                 
                  for taskN in to_schedule:
                    for taskO in old_tasks:
                      if taskO.task_id == taskN.task_id:
                        newF = newF + 1
                     
                    if (newF ==0): # task is not presented in the old set
                      new_tasks.append(taskN)
                      #print "added"
                      #print taskN
                      newF = 0
                    else: #is presented
                      newF = 0

                  
                  for taskD in all_throwen:
                    for taskO in old_tasks:
                      if taskD.task_id == taskO.task_id: #droped task is in old task, we need to remove it
                        self.execution_schedule.remove_task_with_id(taskO.task_id)
                        #print taskO

                  
                  self.execution_schedule.add_new_tasks(new_tasks)
                  self.execution_schedule.set_schedule(to_schedule)

                  rospy.loginfo('Added %s tasks into the schedule to get total of %s' % (len(new_tasks), self.execution_schedule.get_execution_queue_length()))
                  self.drop_tasks(all_throwen, description = "Dropped because it was not possible to find a schedule given the constraints of all tasks. This was dropped because it was randomly selected from the lowest priority set.")

                  return True, new_tasks
                else:  #there are no old tasks 
                      
                  self.execution_schedule.add_new_tasks(to_schedule)                
                  self.execution_schedule.set_schedule(to_schedule)
                  self.drop_tasks(all_throwen, description = "Dropped because it was not possible to find a schedule given the constraints of all tasks. This was dropped because it was randomly selected from the lowest priority set.")                

                  rospy.loginfo('Added %s tasks into the schedule to get total of %s' % (len(to_schedule), self.execution_schedule.get_execution_queue_length()))
                  return True, to_schedule  
                                    

               
      
      else:
        return True,[] #we didnt have anything to schedule
  
    def schedule_tasks(self):
        loopSecs = 5
        
        while not rospy.is_shutdown() and self.running:      

            # all encompassing try/catch to make sure this loop does not go down        
            try:

                # print "scheduling thread %s" % rospy.is_shutdown()      
                try:
                    unscheduled = []
                    # block until at least one task is available
                    unscheduled.append(self.unscheduled_tasks.get(True, loopSecs))
                    # now check for any remaining tasks in the queue
                    try:
                        while True:
                            unscheduled.append(self.unscheduled_tasks.get(False))
                    except Empty, e:
                        pass
                    
                    if self.running: #Lenka note: is this if needed? as while loop has same condition?
                        rospy.loginfo('Got a further %s tasks to schedule' % len(unscheduled))
                        self.try_schedule(unscheduled)                
                    else:
                        rospy.loginfo('Putting %s tasks to schedule later' % len(unscheduled))
                        for task in unscheduled:
                            self.unscheduled_tasks.put(task)

                except Empty, e:
                    # rospy.logdebug('No new tasks to schedule')
                    pass

                self.publish_schedule()

            except Exception, e:
                rospy.logwarn('Caught exception in schedule_tasks loop: %s' % e)
                rospy.sleep(1)


    def execute_tasks(self):
        wait_time = 1 # 1second
        
        while not rospy.is_shutdown() and self.running:           

            # all encompassing try/catch to make sure this loop does not go down
            try:

                # print "executing thread %s" % rospy.is_shutdown()
                if(self.execution_schedule.wait_for_execution_change(wait_time)):
                    if self.running:
                        next_task = self.execution_schedule.get_current_task()
                        if next_task is not None:
                            rospy.loginfo('Next task to execute: %s' % next_task.task_id)
                            self.execute_task(next_task)                
                        else:
                            rospy.logwarn('Next task was None')

            except Exception, e:
                rospy.logwarn('Caught exception in execute_tasks loop: %s' % e)
                rospy.sleep(1)


    def cancel_task(self, task_id): #Lenka note: 
        """ Called when a request is received to cancel a task. The currently executing one is checked elsewhere. """
        if self.execution_schedule.remove_task_with_id(task_id):
            # reschedule after a successful removal
            self.try_schedule([])
            return True
        else:
            return False

    def clear_schedule(self):
        """ Called to clear all tasks from schedule, with the exception of the currently executing one. """
        self.execution_schedule.clear_schedule()



if __name__ == '__main__':
    executor = ScheduledTaskExecutor()        
    rospy.spin()


# create a schedule class which handles blocking until execution and manages the various changes

