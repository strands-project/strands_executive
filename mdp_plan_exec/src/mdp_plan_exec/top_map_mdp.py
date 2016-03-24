from copy import deepcopy
from mdp import Mdp, MdpTransitionDef, MdpPropDef
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from strands_navigation_msgs.srv import GetTopologicalMap, PredictEdgeState
from strands_executive_msgs.msg import MdpAction, MdpActionOutcome, StringIntPair, StringTriple
import strands_executive_msgs.mdp_action_utils as mau 
from mongodb_store.message_store import MessageStoreProxy
from mongodb_store_msgs.msg import StringPair


class TopMapMdp(Mdp):
    def __init__(self,top_map_name, explicit_doors=False, forget_doors=False):
        Mdp.__init__(self)
        forget_doors=True
        got_service=False
        while not got_service:
            try:
                rospy.wait_for_service("/topological_map_publisher/get_topological_map", 1)
                got_service=True
            except rospy.ROSException,e:
                rospy.loginfo("Waiting for get_topological_map service...")
            if rospy.is_shutdown():
                return            
        got_service=False
        while not got_service:
            try:
                rospy.wait_for_service("/topological_prediction/predict_edges", 1)
                got_service=True
            except rospy.ROSException,e:
                rospy.loginfo("Waiting for predict_edges service...")
            if rospy.is_shutdown():
                return
        self.mongo=MessageStoreProxy()
        
        self.top_map_name=top_map_name
        self.get_top_map_srv=rospy.ServiceProxy("/topological_map_publisher/get_topological_map", GetTopologicalMap)
        self.get_edge_estimates=rospy.ServiceProxy("/topological_prediction/predict_edges", PredictEdgeState)        

        self.top_map=self.get_top_map_srv(self.top_map_name).map
        self.check_spaces_in_top_map()
        
        self.nav_actions=[]
        self.door_pass_action="door_wait_and_pass"
        self.door_transitions=[]
        self.create_top_map_mdp_structure()
        
        self.action_descriptions={}
        
       
        
        
        #DOOR MODELLING
        if explicit_doors:  
            self.n_door_edges=0
            self.add_door_model(forget_doors)
                    
        rospy.loginfo("Topological MDP initialised")

    def check_spaces_in_top_map(self):
        for node in self.top_map.nodes:
            if ' ' in node.name:
                raise SyntaxError("The topological node name '" + node.name + "' has a white space. Remove it")
            for edge in node.edges:
                if ' ' in edge.edge_id:
                    raise SyntaxError("An edge from '" + node.name + "' has an edge_id '" + edge.edge_id + "' with a white space. Remove it")

    def create_top_map_mdp_structure(self):
        self.n_state_vars=1
        self.state_vars=['waypoint']
        self.initial_state={'waypoint':0}
        n_waypoints=len(self.top_map.nodes)
        self.state_vars_range={'waypoint':(0,n_waypoints-1)}
                
        self.n_props=n_waypoints
        self.props=[]
        
        self.reward_names=['time']
        
        i=0
        for node in self.top_map.nodes:
            waypoint_name=node.name
            self.props.append(waypoint_name)            
            self.props_def[waypoint_name]=MdpPropDef(name=waypoint_name,
                                                    conds={'waypoint':i})
            i=i+1
        
        i=0
        self.n_actions=0
        for node in self.top_map.nodes:
            source_name=node.name
            for edge in node.edges:
                target_index=self.props.index(edge.node)
                action_name=edge.edge_id
                self.actions.append(action_name)
                self.nav_actions.append(action_name)
                trans=MdpTransitionDef(action_name=action_name,
                                                  pre_conds={'waypoint':i},
                                                  prob_post_conds=[(1.0, {'waypoint':target_index})],
                                                  rewards={'time':1.0})
                self.transitions.append(trans)
                if edge.action == self.door_pass_action:
                    self.door_transitions.append(trans)
                self.n_actions=self.n_actions+1 # all actions have a different name
            i=i+1
            
        
        

    def parse_sta_to_waypoints(self, sta_file, n_states):
        state_vector_names=[None]*n_states
        sta=open(sta_file)
        sta.readline()
        for line in sta:
            line=line.split(':')
            state_vector_names[int(line[0])]=self.get_waypoint_prop(int(line[1][1:-2]))
        return state_vector_names
    
    def get_waypoint_prop(self, waypoint_var_val):
        for key, value  in self.props_def.iteritems():
            if value.conds['waypoint']==waypoint_var_val:
                return key
        rospy.logerr("Waypoint not found!")
        
    def get_waypoint_var_val(self, waypoint_prop):
        return self.props_def[waypoint_prop].conds['waypoint']
    
    def get_waypoint_pose_argument(self, waypoint):
        for node in self.top_map.nodes:
            if node.name==waypoint:
                return self.mongo.insert(PoseStamped(pose=node.pose))
            
        
    
    def set_mdp_action_durations(self, file_name, epoch=None):
        if epoch is None:
            epoch=rospy.Time.now()
        predictions=self.get_edge_estimates(epoch)
        if len(predictions.edge_ids) != self.n_actions:
            rospy.logwarn("Did not receive travel time estimations for all edges, the total navigatio expected values will not be correct")
        for (edge, prob, duration) in zip(predictions.edge_ids, predictions.probs, predictions.durations):
            for transition in self.transitions:
                if edge == transition.action_name:
                    transition.prob_post_conds=[(prob, dict(transition.prob_post_conds[0][1])), (1-prob, dict(transition.pre_conds))]
                    transition.rewards["time"]=duration.to_sec()       
        self.write_prism_model(file_name)
        

    def set_initial_state_from_waypoint(self,current_waypoint):
        self.initial_state['waypoint']=self.props_def[current_waypoint].conds['waypoint']
        
    def target_in_topological_map(self, waypoint):
        return waypoint in [node.name for node in self.top_map.nodes]
    
    def add_door_check_action_description(self, source_wp, target_wp, action_name, door_state_var_name):        
        action_description=MdpAction()
        action_description.name=action_name
        action_description.action_server="door_wait"
        action_description.waypoints=[source_wp]
        action_description.pre_conds=[StringIntPair(string_data=door_state_var_name, int_data=-1)]
        pose_id=self.get_waypoint_pose_argument(target_wp)
        mau.add_object_id_argument(action_description, pose_id, PoseStamped)
        mau.add_float_argument(action_description, 120)
        outcome=MdpActionOutcome()
        outcome.probability=0.9
        outcome.post_conds=[StringIntPair(string_data=door_state_var_name, int_data=1)]
        outcome.duration_probs=[1]
        outcome.durations=[60]
        outcome.status=[GoalStatus.SUCCEEDED]
        outcome.result=[StringTriple(attribute="open", type=MdpActionOutcome.BOOL_TYPE, value="True")]
        action_description.outcomes.append(outcome)
        outcome=MdpActionOutcome()
        outcome.probability=0.1
        outcome.post_conds=[StringIntPair(string_data=door_state_var_name, int_data=0)]
        outcome.duration_probs=[1]
        outcome.durations=[120]
        outcome.status=[GoalStatus.SUCCEEDED]
        outcome.result=[StringTriple(attribute="open", type=MdpActionOutcome.BOOL_TYPE, value="False")]
        action_description.outcomes.append(outcome)
        self.action_descriptions[action_name]=action_description
    
    
    def add_door_model(self, forget_doors):
        rospy.loginfo("Adding doors")
        door_targets=[]
        for i in range(0,len(self.door_transitions)):
            transition = self.door_transitions[i]
            #add stuff to model
            source=transition.pre_conds['waypoint']
            target=transition.prob_post_conds[0][1]['waypoint']
            source_wp=self.get_waypoint_prop(source)
            target_wp=self.get_waypoint_prop(target)
            if source in door_targets:
                index=door_targets.index(source)
                var_name="door_edge" + str(index)
                check_door_action_name='check_door' + str(index) + '_at_' + source_wp 
            else:
                door_targets.append(target)
                var_name="door_edge"+str(self.n_door_edges)
                print var_name
                self.state_vars.append(var_name)
                self.state_vars_range[var_name]=(-1,1) #-1, unkown, 0 closed, 1 open
                self.initial_state[var_name]=-1
                check_door_action_name="check_door" + str(self.n_door_edges) + '_at_' + source_wp 
                self.actions.append(check_door_action_name)
                self.n_actions=self.n_actions+1
                self.n_door_edges=self.n_door_edges+1
            #update transition <- can only pass if door is open
            transition.pre_conds[var_name]=1            
            if forget_doors:
                #reset door value to -1
                for j in range(0, len(transition.prob_post_conds)):
                    transition.prob_post_conds[j][1][var_name]=-1
            #create check transition                
            self.transitions.append(MdpTransitionDef(action_name=check_door_action_name,
                                        pre_conds={'waypoint':source, var_name:-1},
                                        prob_post_conds=[[0.1,{'waypoint':source, var_name:0}],[0.9,{'waypoint':source, var_name:1}]],
                                        rewards={'time':120*0.1+60*0.9}))
            #add action description
            self.add_door_check_action_description(source_wp, target_wp, check_door_action_name, var_name)
            
            #make all other nav transitions from source forget if door was open
            if forget_doors:
                extra_trans_list=[]
                for transition in self.transitions:
                    if transition not in self.door_transitions and transition.action_name in self.nav_actions and transition.pre_conds["waypoint"]==source:
                        trans_closed=deepcopy(transition)
                        trans_open=deepcopy(transition)
                        transition.pre_conds[var_name]=-1
                        trans_closed.pre_conds[var_name]=0
                        trans_open.pre_conds[var_name]=1
                        for j in range(0, len(transition.prob_post_conds)):
                            transition.prob_post_conds[j][1][var_name]=-1
                            trans_closed.prob_post_conds[j][1][var_name]=0
                            trans_open.prob_post_conds[j][1][var_name]=-1
                            extra_trans_list+=[trans_open, trans_closed]
                self.transitions+=extra_trans_list
            
            
            
            
    def set_open_door_probabilities(self, file_name, epoch=None):
        if epoch is not None:
            rospy.loginfo("Getting probabilities for specific time of day")
        else:
            rospy.loginfo("Getting door probabilities for now")
        #TODO change door transition probabilities
        self.write_prism_model(file_name)
        

    def string_int_pair_list_to_dict(self, string_int_pair_list):
        res={}
        for string_int_pair in string_int_pair_list:
            res[string_int_pair.string_data]=string_int_pair.int_data
        return res
    
    def action_msg_to_transition(self, action_msg, waypoint, use_expected_duration):
        pre_conds=self.string_int_pair_list_to_dict(action_msg.pre_conds)
        if waypoint is not None:
            pre_conds["waypoint"]=self.get_waypoint_var_val(waypoint)
        post_conds=[]
        total_prob=0
        for outcome in action_msg.outcomes:
            total_prob+=outcome.probability
            post_cond=self.string_int_pair_list_to_dict(outcome.post_conds)
            if outcome.waypoint != '':
                post_cond["waypoint"]=self.get_waypoint_var_val(outcome.waypoint)
            post_conds.append([outcome.probability, post_cond])
        if total_prob !=1:
            rospy.logerr("Probabilities of action outcomes don't add up to 1")
        
        if use_expected_duration:
            expected_duration=0
            for outcome in action_msg.outcomes:
                for (dur_prob, dur_val) in zip(outcome.duration_probs, outcome.durations):
                    expected_duration+=outcome.probability*dur_prob*dur_val
        else:
            rospy.logerr("Clustered times not supported yet...")
                    
        return MdpTransitionDef(action_name=action_msg.name,
                                    pre_conds=pre_conds,
                                    prob_post_conds=post_conds,
                                    rewards={"time":expected_duration})
    
    
    def add_extra_domain(self, var_list, action_list, use_expected_duration=True):
        for var in var_list:
            self.n_state_vars+=1
            self.state_vars.append(var.name)
            self.initial_state[var.name]=var.init_val
            self.state_vars_range[var.name]=[var.min_range, var.max_range]
        
        
        for action in action_list:
            self.action_descriptions.update({action.name:action})
            if action.waypoints==[]:
                self.transitions.append(self.action_msg_to_transition(action, None, use_expected_duration))
            for waypoint in action.waypoints:
                self.transitions.append(self.action_msg_to_transition(action, waypoint, use_expected_duration))
                                            
                                       
        



