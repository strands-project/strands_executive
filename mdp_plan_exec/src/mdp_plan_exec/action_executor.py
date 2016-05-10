import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from mongodb_store.message_store import MessageStoreProxy
import mongodb_store.util as dc_util
from strands_executive_msgs.msg import MdpAction, Task


class ActionExecutor(object):
    def __init__(self):
        self.msg_store = MessageStoreProxy() 
        self.cancelled=False
    
    def get_action_types(self, action_server):
        """ 
        Returns the type string related to the action string provided.
        """
        rospy.logdebug("Action server provided: %s", action_server)
        topics = rospy.get_published_topics(action_server)
        for [topic, type] in topics:            
            if topic.endswith('feedback'):
                return (type[:-8], type[:-14] + 'Goal')
        raise RuntimeError('No action associated with topic: %s'% action_server)
        
    def instantiate_from_string_pair(self, string_pair):
        if string_pair[0] == Task.STRING_TYPE:
            return string_pair[1]
        elif string_pair[0] == Task.INT_TYPE:
            return int(string_pair[1])
        elif string_pair[0] == Task.FLOAT_TYPE:
            return float(string_pair[1])     
        elif string_pair[0] ==Task.TIME_TYPE:
            return rospy.Time.from_sec(float(string_pair[1]))
        elif string_pair[0] == Task.DURATION_TYPE:
            return rospy.Duration.from_sec(float(string_pair[1]))
        elif string_pair[0] == Task.BOOL_TYPE:   
            return string_pair[1] == 'True'
        else:           

            msg = self.msg_store.query_id(string_pair[1], string_pair[0])[0]
            # print msg
            if msg == None:
                raise RuntimeError("No matching object for id %s of type %s" % (string_pair[1], string_pair[0]))
            return msg

    def get_arguments(self, argument_list):
        res=[]
        for string_pair in argument_list:
            res.append(self.instantiate_from_string_pair([string_pair.first, string_pair.second]))
        return res
 
    
    def get_max_action_duration(self, action_outcomes):
        res=0
        for action_outcome_msg in action_outcomes:
            for duration in action_outcome_msg.durations:
                res=max(res, duration)
        return res
    
    def get_max_prob_outcome(self, action_outcomes):
        max_prob=0
        res=None
        for action_outcome_msg in action_outcomes:
            if action_outcome_msg.probability > max_prob:
                max_prob=action_outcome_msg.probability
                res=action_outcome_msg
        return res
    
    def execute_action(self, action_msg):
        self.cancelled=False
        rospy.loginfo("Executing " + action_msg.name + ". Actionlib server is: " + action_msg.action_server)
        if action_msg.action_server != '':
            try:
                (action_string, goal_string) = self.get_action_types(action_msg.action_server)
                action_clz = dc_util.load_class(dc_util.type_to_class_string(action_string))
                goal_clz = dc_util.load_class(dc_util.type_to_class_string(goal_string))

                argument_list = self.get_arguments(action_msg.arguments)
                goal = goal_clz(*argument_list) 
                
                poll_wait = rospy.Duration(1)

                max_action_duration=self.get_max_action_duration(action_msg.outcomes)
                wiggle_room=30
                action_client=actionlib.SimpleActionClient(action_msg.action_server, action_clz)
                action_client.wait_for_server(poll_wait)
                action_client.send_goal(goal)
                
                action_finished = False
                timer=0
                while (not action_finished) and (timer < max_action_duration + wiggle_room) and (not self.cancelled):
                    timer += poll_wait.to_sec()
                    action_finished = action_client.wait_for_result(poll_wait)
                    
                if not action_finished:
                    if self.cancelled:
                        rospy.logwarn("Policy execution has been preempted by another process")
                    else:
                        rospy.logwarn('Action %s exceeded maximum duration, preempting' % action_msg.name)
                    action_client.cancel_all_goals()
                    action_finished = action_client.wait_for_result(rospy.Duration(60)) #give some time for action to cancel
                    if not action_finished:
                        rospy.logwarn('Action %s did not respond to preemption, carrying on regardless. This could have dangerous side effects.' % action_msg.name)


                status=action_client.get_state()    
                result=action_client.get_result()
                print GoalStatus.to_string(status)
                print result
                
            except Exception, e:
                rospy.logwarn('Caught exception when trying to execute the action: %s' % e)
                status=GoalStatus.ABORTED
                result=None
   
        else:
            status=GoalStatus.SUCCEEDED
            result=None
        if self.cancelled:
            self.cancelled = False
            return (None, None)
        else:
            return (status, self.build_update_state_dict(status, result, action_msg.outcomes))
    
    def build_update_state_dict(self, status, result, action_outcomes):
        for action_outcome_msg in action_outcomes:
            if action_outcome_msg.probability==1:
                return self.build_dict_from_string_int_pairs(action_outcome_msg.post_conds)
            elif action_outcome_msg.status==[] or status in action_outcome_msg.status:
                if action_outcome_msg.result==[]:
                    return self.build_dict_from_string_int_pairs(action_outcome_msg.post_conds)
                else:
                    is_match=True
                    for string_triple in action_outcome_msg.result:
                        field_val=self.instantiate_from_string_pair([string_triple.type, string_triple.value])
                        if result.__getattribute__(string_triple.attribute)!=field_val:
                            is_match=False
                            break
                    if is_match:
                        return self.build_dict_from_string_int_pairs(action_outcome_msg.post_conds)
                
        rospy.logwarn("Didn't find an outcome matching the action result! Going to use the most probable outcome...")
        print action_outcomes
        action_outcome_msg=self.get_max_prob_outcome(action_outcomes)
        print action_outcome_msg
        return self.build_dict_from_string_int_pairs(action_outcome_msg.post_conds)

            
    def build_dict_from_string_int_pairs(self, string_int_pairs):
        res={}
        for string_int_pair in string_int_pairs:
            res[string_int_pair.string_data]=string_int_pair.int_data
        return res    
            
    
    def cancel_all_goals(self):
        self.cancelled=True