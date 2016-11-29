#! /usr/bin/env python

import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from strands_executive_msgs.msg import ExecutePolicyExtendedAction, ExecutePolicyExtendedFeedback, ExecutePolicyExtendedGoal, MdpStateVar, StringIntPair, StringTriple, MdpAction, MdpActionOutcome, MdpDomainSpec, MdpTask, Task
from strands_executive_msgs.srv import GetGuaranteesForCoSafeTask, GetGuaranteesForCoSafeTaskRequest
import strands_executive_msgs.mdp_action_utils as mau
from strands_executive_msgs.srv import AddCoSafeTasks, SetExecutionStatus, DemandCoSafeTask

import sys

def create_metric_map_action(waypoint_name, duration=5):
    action_name='wait_at_' + waypoint_name
    state_var_name="executed_" + action_name
    
    # set state_var
    var=MdpStateVar(name=state_var_name,
                    init_val=0,
                    min_range=0,
                    max_range=1)
    
    
    # set action
    outcome=MdpActionOutcome(probability=1.0,
                             #waypoint=[], #same waypoint
                             post_conds=[StringIntPair(string_data=state_var_name, int_data=1)],
                             duration_probs=[1.0],
                             durations=[duration],
                             #status=GoalStatus #prob 1, isnt needed
                             #result=[] #prob 1, isnt needed
                             )

    action = MdpAction(name=action_name, 
                     action_server='wait_action', 
                     waypoints=[waypoint_name],
                     pre_conds=[StringIntPair(string_data=state_var_name, int_data=0)],
                     outcomes=[outcome]
                     )

    mau.add_time_argument(action, rospy.Time())
    mau.add_duration_argument(action, rospy.Duration(duration))

    
    return (var, action)


def get_services():
    # get services necessary to do the jon
    add_tasks_srv_name = '/task_executor/add_co_safe_tasks'
    demand_task_srv_name = '/task_executor/demand_co_safe_task'
    set_exe_stat_srv_name = '/task_executor/set_execution_status'
    rospy.loginfo("Waiting for task_executor service...")
    rospy.wait_for_service(add_tasks_srv_name)
    rospy.wait_for_service(set_exe_stat_srv_name)
    rospy.loginfo("Done")        
    add_tasks_srv = rospy.ServiceProxy(add_tasks_srv_name, AddCoSafeTasks)
    demand_task_srv = rospy.ServiceProxy(demand_task_srv_name, DemandCoSafeTask)
    set_execution_status = rospy.ServiceProxy(set_exe_stat_srv_name, SetExecutionStatus)
    return add_tasks_srv, demand_task_srv, set_execution_status

if __name__ == '__main__':
    rospy.init_node('mdp_client_test')
    
    n_waypoints=3
    
      # get services to call into execution framework
    add_tasks, demand_task, set_execution_status = get_services()

    
    
    spec=MdpDomainSpec()
    ltl_task=''
    for i in range(1, n_waypoints+1):
        waypoint_name="WayPoint" + str(i)
        (var, action)=create_metric_map_action(waypoint_name)
        spec.vars.append(var)
        spec.actions.append(action)
        ltl_task+='(F executed_wait_at_' + waypoint_name + '=1) & '
    spec.ltl_task=ltl_task[:-3]
    
    # print add_tasks([spec],[rospy.Time()], [rospy.get_rostime() + rospy.Duration(60 * 60)])
    set_execution_status(True)

    task = MdpTask()
    task.mdp_spec = spec
    task.start_after = rospy.get_rostime()
    task.end_before = task.start_after + rospy.Duration(60 * 60)
    task.priority = Task.HIGH_PRIORITY
    task.is_interruptible = True
    print add_tasks([task])