from mdp import Mdp, MdpTransitionDef, MdpPropDef
#from strands_navigation_msgs.msg import TopologicalMap
import rospy
from strands_navigation_msgs.srv import GetTopologicalMap


class TopMapMdp(Mdp):
    def __init__(self,top_map_name):
        Mdp.__init__(self)
        
        
        self.top_map_name=top_map_name
        self.get_top_map_srv=rospy.ServiceProxy("/topological_map_publisher/get_topological_map", GetTopologicalMap)

        self.top_map=self.get_top_map_srv(self.top_map_name).map        
        self.create_top_map_mdp_structure()
        

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

    #def get_fremen_stats(epoch):
        


 

        
    #def set_initial_state_from_name(self,state_name):
        #index=self.state_names.index(state_name)
        #self.set_initial_state(index)

