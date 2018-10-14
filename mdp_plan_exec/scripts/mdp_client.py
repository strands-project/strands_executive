#! /usr/bin/env python

import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from strands_executive_msgs.msg import ExecutePolicyAction, ExecutePolicyFeedback, ExecutePolicyGoal, MdpStateVar, StringIntPair, StringTriple, MdpAction, MdpActionOutcome, MdpDomainSpec
from strands_executive_msgs.srv import GetGuaranteesForCoSafeTask, GetGuaranteesForCoSafeTaskRequest
import strands_executive_msgs.mdp_action_utils as mau

def add_door(spec): #EXAMPLE OF ADDING AN ACTION WITH UNCERTAIN OUTCOME. NOTE THAT I ADD ACTIONS SIMILAR TO THIS ONE IN THE MDP ANYWAY, JUST AN EXAMPLE
    door_var=MdpStateVar(name="door1",
                      init_val=-1,
                      min_range=-1,
                      max_range=1)
    
    door_action_outcomes=[]
    outcome=MdpActionOutcome(probability=0.8,
                             post_conds=[StringIntPair(string_data="door1", int_data=1)],
                             duration_probs=[1],
                             durations=[2],
                             status=[],
                             result=[StringTriple(attribute='open', type=MdpActionOutcome.BOOL_TYPE, value='True')]
                             )
    door_action_outcomes.append(outcome)
    
    outcome=MdpActionOutcome(probability=0.2,
                             post_conds=[StringIntPair(string_data="door1", int_data=0)],
                             duration_probs=[1],
                             durations=[2],
                             status=[],
                             result=[StringTriple(attribute='open', type=MdpActionOutcome.BOOL_TYPE, value='False')]
                             )
    door_action_outcomes.append(outcome)
    
    
    
    door_action=MdpAction(name='check_the_door',
                          action_server='door_check',
                          arguments=[],
                          waypoints=["WayPoint1", "WayPoint9"],
                          pre_conds=[StringIntPair(string_data="door1", int_data=-1)],
                          outcomes=door_action_outcomes)
    
    
    spec.vars.append(door_var)
    spec.actions.append(door_action)
    return spec



def create_metric_map_action(waypoint_name, duration=4*60):
    action_name='metric_map_at_' + waypoint_name
    state_var_name="executed_" + action_name
    
    #set state_var
    var=MdpStateVar(name=state_var_name,
                    init_val=0,
                    min_range=0,
                    max_range=1)
    
    
    #set action
    outcome=MdpActionOutcome(probability=1.0,
                             #waypoint=[], #same waypoint
                             post_conds=[StringIntPair(string_data=state_var_name, int_data=1)],
                             duration_probs=[1.0],
                             durations=[duration],
                             #status=GoalStatus #prob 1, isnt needed
                             #result=[] #prob 1, isnt needed
                             )
    action = MdpAction(name=action_name, 
                     action_server='ptu_pan_tilt_metric_map', 
                     waypoints=[waypoint_name],
                     pre_conds=[StringIntPair(string_data=state_var_name, int_data=0)],
                     outcomes=[outcome]
                     )
    mau.add_int_argument(action, '-160')
    mau.add_int_argument(action, '20')
    mau.add_int_argument(action, '160')
    mau.add_int_argument(action, '-30')
    mau.add_int_argument(action, '30')
    mau.add_int_argument(action, '30')
    return (var, action)

def feedback_cb(feedback):
    print("Got Feedback: " + str(feedback))

if __name__ == '__main__':
    rospy.init_node('mdp_client_test')
    
    
    waypoints = [2, 3, 21]
    
    mdp_ac=actionlib.SimpleActionClient("mdp_plan_exec/execute_policy", ExecutePolicyAction)
    
    mdp_ac.wait_for_server()
    goal=ExecutePolicyGoal()
    
    mdp_estimates=rospy.ServiceProxy("mdp_plan_exec/get_guarantees_for_co_safe_task", GetGuaranteesForCoSafeTask)
    request=GetGuaranteesForCoSafeTaskRequest()
    
    
    
    spec=MdpDomainSpec()
    ltl_task=''
    #for i in waypoints:
        #waypoint_name="WayPoint" + str(i)
        #(var, action)=create_metric_map_action(waypoint_name)
        #spec.vars.append(var)
        #spec.actions.append(action)
        #ltl_task+='(F executed_metric_map_at_' + waypoint_name + '=1) & '
    #spec.ltl_task=ltl_task[:-3]
    #spec.ltl_task='(F executed_metric_map_at_WayPoint10=1) & (F executed_metric_map_at_WayPoint5=1) & (F executed_metric_map_at_WayPoint7=1) & (F executed_metric_map_at_WayPoint1=1)'
    
    #spec.ltl_task='(F ("WayPoint3" & (X "WayPoint4"))) & ((!"WayPoint3") U executed_metric_map_at_WayPoint1=1) & (F executed_metric_map_at_WayPoint2=1) & (F executed_metric_map_at_WayPoint3=1) & (F executed_metric_map_at_WayPoint4=1)'
   # spec.ltl_task='(F executed_metric_map_at_WayPoint2=1) & (F executed_metric_map_at_WayPoint3=1) & (F executed_metric_map_at_WayPoint21=1)'
    spec.ltl_task='(F "WayPoint3")'

    request.epoch = rospy.Time.now()
    request.spec=spec
    request.initial_waypoint="WayPoint5"
    service_response=mdp_estimates(request)
    print(service_response)
    
    #goal.spec=spec
    #mdp_ac.send_goal(goal, feedback_cb = feedback_cb)
    ##mdp_ac.wait_for_result(rospy.Duration(10))
    ##mdp_ac.cancel_all_goals()
    #mdp_ac.wait_for_result()  
    #print(GoalStatus.to_string(mdp_ac.get_state()))
    
    
   
   
   

    
    
    
    
    