from mdp import Mdp, MdpTransitionDef, MdpPropDef
import rospy
import actionlib
from strands_navigation_msgs.srv import GetTopologicalMap, PredictEdgeState


class TopMapMdp(Mdp):
    def __init__(self,top_map_name, explicit_doors=False, forget_doors=False):
        Mdp.__init__(self)
        
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
        
        self.top_map_name=top_map_name
        self.get_top_map_srv=rospy.ServiceProxy("/topological_map_publisher/get_topological_map", GetTopologicalMap)
        self.get_edge_estimates=rospy.ServiceProxy("/topological_prediction/predict_edges", PredictEdgeState)        

        self.top_map=self.get_top_map_srv(self.top_map_name).map
        self.check_spaces_in_top_map()
        
       
        self.door_transitions=[]
        self.create_top_map_mdp_structure()
        
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
                trans=MdpTransitionDef(action_name=action_name,
                                                  pre_conds={'waypoint':i},
                                                  prob_post_conds=[(1.0, {'waypoint':target_index})],
                                                  rewards={'time':1.0},
                                                  exec_count=0)
                self.transitions.append(trans)
                if edge.action == "doorPassing":
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
    
    def set_mdp_action_durations(self, file_name, epoch=None):
        if epoch is None:
            epoch=rospy.Time.now()
        try:
            predictions=self.get_edge_estimates(epoch)
            if len(predictions.edge_ids) != self.n_actions:
                rospy.logwarn("Did not receive travel time estimations for all edges, the total navigation expected values will not be correct")
            for (edge, prob, duration) in zip(predictions.edge_ids, predictions.probs, predictions.durations):
                index=self.actions.index(edge)
                transition=self.transitions[index]
                self.transitions[index].prob_post_conds=[(prob, dict(transition.prob_post_conds[0][1])), (1-prob, dict(transition.pre_conds))]
                self.transitions[index].rewards["time"]=duration.to_sec()
        except rospy.ServiceException, e:
            rospy.logwarn("Error calling edge transversal times prediction service: " + str(e))
            rospy.logwarn("The total navigation expected values will not be for the requested epoch.")
        self.write_prism_model(file_name)
        

    def set_initial_state_from_waypoint(self,current_waypoint):
        self.initial_state['waypoint']=self.props_def[current_waypoint].conds['waypoint']
        
    def target_in_topological_map(self, waypoint):
        return waypoint in [node.name for node in self.top_map.nodes]
    
    def add_door_model(self, forget_doors):
        rospy.loginfo("Adding doors")
        self.reward_names.append("info")
        door_targets=[]
        for i in range(0,len(self.door_transitions)):
            transition = self.door_transitions[i]
            #add stuff to model
            source=transition.pre_conds['waypoint']
            target=transition.prob_post_conds[0][1]['waypoint']
            if source in door_targets:
                index=door_targets.index(source)
                var_name="door_edge" + str(index)
                check_door_action_name='check_door' + str(index)
            else:
                door_targets.append(target)
                var_name="door_edge"+str(self.n_door_edges)
                print var_name
                self.state_vars.append(var_name)
                self.state_vars_range[var_name]=(-1,1) #-1, unkown, 0 closed, 1 open
                self.initial_state[var_name]=-1
                check_door_action_name="check_door"+str(self.n_door_edges)
                self.actions.append(check_door_action_name)
                self.n_actions=self.n_actions+1
                self.n_door_edges=self.n_door_edges+1
                #update transition <- can only pass if door is open
            transition.pre_conds[var_name]=1
            if forget_doors:
                #reset dooo value to -1
                for j in range(0, len(transition.prob_post_conds)):
                    transition.prob_post_conds[j][1][var_name]=-1
            #create check transition                
            self.transitions.append(MdpTransitionDef(action_name=check_door_action_name,
                                        pre_conds={'waypoint':source, var_name:-1},
                                        prob_post_conds=[[0.1,{'waypoint':source, var_name:0}],[0.9,{'waypoint':source, var_name:1}]],
                                        rewards={'time':0.1, 'info':1},
                                        exec_count=0))
            
    def set_open_door_probabilities(self, file_name, epoch=None):
        if epoch is not None:
            rospy.loginfo("Getting probabilities for specific time of day")
        else:
            rospy.loginfo("Getting door probabilities for now")
        #TODO change door transition probabilities
        self.write_prism_model(file_name)

