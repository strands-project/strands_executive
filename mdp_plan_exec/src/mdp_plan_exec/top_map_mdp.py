from mdp import Mdp, MdpTransitionDef, MdpPropDef
import rospy
import actionlib
from strands_navigation_msgs.srv import GetTopologicalMap, PredictEdgeState


class TopMapMdp(Mdp):
    def __init__(self,top_map_name):
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
        self.create_top_map_mdp_structure()
                    
        rospy.loginfo("Topological MDP initialised")

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
                self.transitions.append(MdpTransitionDef(action_name=action_name,
                                                  pre_conds={'waypoint':i},
                                                  prob_post_conds=[(1.0, {'waypoint':target_index})],
                                                  rewards={'time':1.0},
                                                  exec_count=0))
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
        predictions=self.get_edge_estimates(epoch)
        if len(predictions.edge_ids) != self.n_actions:
            rospy.lowarn("Did not receive travel time estimations for all edges, the total navigatio expected values will not be correct")
        for (edge, prob, duration) in zip(predictions.edge_ids, predictions.probs, predictions.durations):
            index=self.actions.index(edge)
            transition=self.transitions[index]
            self.transitions[index].prob_post_conds=[(prob, dict(transition.prob_post_conds[0][1])), (1-prob, dict(transition.pre_conds))]
            self.transitions[index].rewards["time"]=duration.to_sec()       
        self.write_prism_model(file_name)
        

    def set_initial_state_from_waypoint(self,current_waypoint):
        self.initial_state=self.props_def[current_waypoint].conds

