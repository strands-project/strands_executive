from mdp import Mdp, MdpTransitionDef, MdpPropDef
import rospy
import actionlib
from strands_navigation_msgs.srv import GetTopologicalMap
from frenap.msg import TopologicalPredictionAction, TopologicalPredictionGoal


class TopMapMdp(Mdp):
    def __init__(self,top_map_name):
        Mdp.__init__(self)
        
        
        self.top_map_name=top_map_name
        self.get_top_map_srv=rospy.ServiceProxy("/topological_map_publisher/get_topological_map", GetTopologicalMap)

        self.top_map=self.get_top_map_srv(self.top_map_name).map        
        self.create_top_map_mdp_structure()
        
        self.fremen_ac=actionlib.SimpleActionClient('/FreNaP', TopologicalPredictionAction)
            
        got_server=self.fremen_ac.wait_for_server(rospy.Duration(1))
        while not got_server:
            rospy.loginfo("Waiting for TopologicalPrediction action.")
            got_server=self.fremen_ac.wait_for_server(rospy.Duration(1))
            
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
    
    def get_fremen_stats(self,epoch):
        goal=TopologicalPredictionGoal(action='build', mapName=self.top_map_name, resultOrder=-1,durationOrder=-1)
        self.fremen_ac.send_goal(goal)
        self.fremen_ac.wait_for_result()
        goal=TopologicalPredictionGoal(action='predict', predictionTime=epoch)
        self.fremen_ac.send_goal(goal)
        self.fremen_ac.wait_for_result()
        result=self.fremen_ac.get_result()
        

    def set_initial_state_from_waypoint(self,current_waypoint):
        self.initial_state=self.props_def[current_waypoint].conds

