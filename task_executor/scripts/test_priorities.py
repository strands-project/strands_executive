#!/usr/bin/env python
import rospy
from strands_executive_msgs import task_utils
from strands_executive_msgs.msg import Task
from strands_executive_msgs.srv import AddTasks, SetExecutionStatus
from strands_navigation_msgs.msg import *
import sys

def get_services():
  # get services necessary to do the jon 
  add_tasks_srv_name = '/task_executor/add_tasks'
  set_exe_stat_srv_name = '/task_executor/set_execution_status'
  rospy.loginfo("Waiting for task_executor service...")
  rospy.wait_for_service(add_tasks_srv_name)
  rospy.wait_for_service(set_exe_stat_srv_name)
  rospy.loginfo("Done")
  add_tasks_srv = rospy.ServiceProxy(add_tasks_srv_name, AddTasks)
  set_execution_status = rospy.ServiceProxy(set_exe_stat_srv_name, SetExecutionStatus)
  return add_tasks_srv, set_execution_status

def create_wait_task(node, start, end, secs=rospy.Duration(5), prio=1):
  
  wait_task = Task(action='wait_action',start_node_id=node, end_node_id=node, max_duration = secs, start_after = start, end_before = end, priority = prio) #2280 passing time for 3 task, 2270 failing
  task_utils.add_time_argument(wait_task, rospy.Time()) # Lenka note: what is this???
  task_utils.add_duration_argument(wait_task, secs)
  return wait_task

if __name__ == '__main__':
  rospy.init_node("test_priorities")
  # get services to call into execution framework
  add_task, set_execution_status = get_services()

  #for simplicity all tasks happens in same locations
  
  test = 12

  #Test A - tasks with same priority, all valid tasks
  #Travel duration is 10!

  if(test==1):
    #Test A1I - all feasible, arriving at the batch
    tasks = []
    st = rospy.Time.now()  
    for i in range(0,5):
      tasks.append(create_wait_task('h_2', st, st+rospy.Duration(65), rospy.Duration(5),2))
    task_id = add_task(tasks)
    set_execution_status(True) # Set the task executor running (if it isn't already)
  
  elif(test==2):
    #Test A2I - only some (3) feasible, arriving at the batch
    tasks = []
    st = rospy.Time.now()  
    for i in range(0,5):
      tasks.append(create_wait_task('h_2', st, st+rospy.Duration(35), rospy.Duration(5),2))
    task_id = add_task(tasks)
    set_execution_status(True) # Set the task executor running (if it isn't already)
  
  elif(test==3):
    #Test A3I - none feasible, arriving at the batch
    tasks = []
    st = rospy.Time.now()  
    for i in range(0,5):
      tasks.append(create_wait_task('h_2', st, st+rospy.Duration(4), rospy.Duration(5),2))
    task_id = add_task(tasks)
    set_execution_status(True) # Set the task executor running (if it isn't already)

  elif(test==4):
    #Test A1II - all feasible, arriving separately
    
    st = rospy.Time.now()  
    for i in range(0,5):
      tasks = []
      tasks.append(create_wait_task('h_2', st, st+rospy.Duration(75), rospy.Duration(5),2))
      task_id = add_task(tasks)
      set_execution_status(True) # Set the task executor running (if it isn't already)
      rospy.sleep(2) 

  elif(test==5):  
    #Test A2II - some (3) feasible, arriving separately
    
    st = rospy.Time.now()  
    for i in range(0,5):
      tasks = []
      tasks.append(create_wait_task('h_2', st, st+rospy.Duration(43), rospy.Duration(5),2))
      task_id = add_task(tasks)
      set_execution_status(True) # Set the task executor running (if it isn't already)
      rospy.sleep(2)


  if(test==6):
    #Test B1I - all feasible, arriving at the batch, different priorities 
    tasks = []
    st = rospy.Time.now()  
    for i in range(0,5):
      tasks.append(create_wait_task('h_2', st, st+rospy.Duration(65), rospy.Duration(5),i+1))
    task_id = add_task(tasks)
    set_execution_status(True) # Set the task executor running (if it isn't already)
  
  elif(test==7):
    #Test B2I - only some (3) feasible, arriving at the batch, different priorities - first ones have higher priority, should be executed, last 2 withdrawen
    tasks = []
    st = rospy.Time.now()  
    for i in range(0,5):
      tasks.append(create_wait_task('h_2', st, st+rospy.Duration(37), rospy.Duration(5),5-i))
    task_id = add_task(tasks)
    set_execution_status(True) # Set the task executor running (if it isn't already)
  
  elif(test==8):
    #Test B3I - none feasible, arriving at the batch, priorities
    tasks = []
    st = rospy.Time.now()  
    for i in range(0,5):
      tasks.append(create_wait_task('h_2', st, st+rospy.Duration(4), rospy.Duration(5),i))
    task_id = add_task(tasks)
    set_execution_status(True) # Set the task executor running (if it isn't already)

  elif(test==9):
    #Test B1II - all feasible, arriving separately
    
    st = rospy.Time.now()  
    for i in range(0,5):
      tasks = []
      tasks.append(create_wait_task('h_2', st, st+rospy.Duration(73), rospy.Duration(5),i))
      task_id = add_task(tasks)
      set_execution_status(True) # Set the task executor running (if it isn't already)
      rospy.sleep(2)

  elif(test==10):
    #Test B2II - some (3) feasible, arriving separately, with higher prio first
    
    st = rospy.Time.now()  
    for i in range(0,5):
      tasks = []
      tasks.append(create_wait_task('h_2', st, st+rospy.Duration(43), rospy.Duration(5),5-i))
      task_id = add_task(tasks)
      set_execution_status(True) # Set the task executor running (if it isn't already)
      rospy.sleep(2)
  
  elif(test==11):
    #Test B2II - some (3) feasible, arriving separately, with lowest prio first
    
    st = rospy.Time.now()  
    for i in range(0,5):
      tasks = []
      tasks.append(create_wait_task('h_2', st, st+rospy.Duration(43), rospy.Duration(5),i+1))
      task_id = add_task(tasks)
      set_execution_status(True) # Set the task executor running (if it isn't already)
      rospy.sleep(2)

  elif(test==12):
    #Test C1 - 5 feasible, arriving in the batch, + one with higher priority overlapping with them
    tasks = []
    st = rospy.Time.now()+rospy.Duration(0)  
    for i in range(0,5):
      tasks.append(create_wait_task('h_2', st, st+rospy.Duration(65), rospy.Duration(5),1))
    task_id = add_task(tasks)
    set_execution_status(True) # Set the task executor running (if it isn't already)
    rospy.sleep(6)
 
    tasks = []
    tasks.append(create_wait_task('h_2', st, st+rospy.Duration(65), rospy.Duration(50),2))

    task_id = add_task(tasks)

 #TODO test if non-interruptable tasks



