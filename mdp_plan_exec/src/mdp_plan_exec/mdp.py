#!/usr/bin/python


import sys
import rospy


from ros_datacentre.message_store import MessageStoreProxy
from strands_navigation_msgs.msg import TopologicalNode
from strands_navigation_msgs.msg import NavStatistics

class Mdp(object):
    
    def __init__(self):
        #list of attributes of an MDP object
        self.top_map=''
        self.initial_state=0
        self.n_states=0
        self.state_names=[]
        self.n_props=0
        self.props=[]
        self.n_actions=0
        self.actions=[]
        self.prop_map=[[]]
        self.transitions=[[]]
        self.transitions_transversal_count=[[]]
        self.rewards=[[]]
        self.current_policy=[]
        
        
       
    def write_prism_model(self,file_name):
        f=open(file_name,'w')
        f.write('mdp\n \n')
        f.write('module M \n \n')
        f.write('s:[0..'+str(self.n_states-1)+'] init ' + str(self.initial_state) + ';\n \n')
        
        
        
        for i in range(0,self.n_states):
            for j in range(0,self.n_actions):
                current_trans_list=self.transitions[i][j]
                if current_trans_list:
                    trans_string='[' + self.actions[j] + '] s=' + str(i) + ' -> '
                    for trans in current_trans_list:
                        trans_string=trans_string + str(trans[1]) + ":(s'=" + str(trans[0]) + ') + '
                    f.write(trans_string[:-3] + ';\n')    
        
        
        f.write('\nendmodule\n\n')
        
        for i in range(0,self.n_props):
            f.write('label "'+ self.props[i] + '" = ')
            prop_string=''
            for j in range(0,self.n_states):
                if self.prop_map[j][i]:
                   prop_string=prop_string + 's=' + str(j) + ' | '
            f.write(prop_string[:-3] + ';\n')  
        
        f.write('\n')
        
  
        #f.write('label "goal" = ')
        
        #goal_states_string=''
        #for goal_state in self.goal_states:
            #goal_states_string=goal_states_string + 's=' + str(goal_state) + ' | '
            
        #f.write(goal_states_string[:-3] + ';\n\n')
        
        f.write('rewards "time"\n')
        
        for i in range(0,self.n_states):
            for j in range(0,self.n_actions):
                if self.rewards[i][j] != 0:
                    f.write('    [' + self.actions[j] + '] s=' + str(i) + ':' + str(self.rewards[i][j]) + ';\n')
        
        f.write('endrewards\n')
        
        f.close()
        
    def set_initial_state(self,initial_state):
        self.initial_state=initial_state
        
    def get_expected_edge_transversal_time(self,state_index,action_name):
        action_index=self.actions.index(action_name)
        return self.rewards[state_index][action_index]
        
    def get_total_transversals(self,state_index,action_name):
        action_index=self.actions.index(action_name)
        return self.transitions_transversal_count[state_index][action_index]
        
        
        
 
    

class TopMapMdp(Mdp):
    def __init__(self,top_map_name):
        
        self.top_map=top_map_name
        
        self.initial_state=0
        
        top_nodes=self.read_top_map()
        
        self.n_states=len(top_nodes)
        self.state_names=[None]*self.n_states
        
        self.n_props=self.n_states
        self.props=[None]*self.n_props
        self.prop_map=[[False]*self.n_props for i in range(self.n_states)]
        
        
        for i in range(0,self.n_props):
            self.prop_map[i][i]=True
            
       
        
        i=0
        for entry in top_nodes:
            self.props[i]=entry[0].name
            self.state_names[i]=entry[0].name
            i=i+1
            
            
        
        self.n_actions=0
        for entry in top_nodes:
            self.n_actions=self.n_actions+len(entry[0].edges)
        

            
        self.rewards=[[0]*self.n_actions for i in range(self.n_states)]
        
        self.transitions=[[False]*self.n_actions for i in range(self.n_states)]
        
        self.transitions_transversal_count=[[0]*self.n_actions for i in range(self.n_states)]
        
        self.actions=[None]*self.n_actions
        

        state_index=0
        action_index=0
        for entry in top_nodes:
            current_edges=entry[0].edges
            for edge in current_edges:
                #if edge.action='cross_door':
                    #add stuff to model
                target_index=self.state_names.index(edge.node)
                self.actions[action_index]='goto_'+self.state_names[state_index] + '_' + edge.node
                self.transitions[state_index][action_index]= [[target_index,1]]
                self.rewards[state_index][action_index]=1
                action_index=action_index+1      
            state_index=state_index+1

        
        

    def read_top_map(self):


        msg_store = MessageStoreProxy(collection='topological_maps')
    
        query_meta = {}
        query_meta["pointset"] =self.top_map
        available = len(msg_store.query(TopologicalNode._type, {}, query_meta)) > 0


        if available <= 0 :
            rospy.logerr("Desired pointset '"+point_set+"' not in datacentre")
            rospy.logerr("Available pointsets: "+str(available))
            raise Exception("Can't find waypoints.")
    
        else :
            query_meta = {}
            query_meta["pointset"] = self.top_map
            message_list = msg_store.query(TopologicalNode._type, {}, query_meta)
    
        return message_list


        
    def update_nav_statistics(self):
        msg_store = MessageStoreProxy()
    
        query_meta = {}
        query_meta["pointset"] = self.top_map
        print self.top_map
        message_list = msg_store.query(NavStatistics._type, {}, query_meta)
        n_data=len(message_list)
        n_unprocessed_data=n_data

        
        
        for i in range(0,self.n_actions):
            current_action=self.actions[i]
            if 'goto' in current_action:
                action_index=self.actions.index(current_action)
                current_action=current_action.split('_')
                source_index=self.state_names.index(current_action[1])
                target_index=self.state_names.index(current_action[2])
                j=0
                n_total_data=1
                expected_time=0
                total_outcomes_count=1
                outcomes_count=[0]*self.n_states
                outcomes_count[target_index]=1
                while j<n_unprocessed_data:
                    entry=message_list[j]
                    if current_action[1]==entry[0].origin and current_action[2]==entry[0].target and not entry[0].final_node == 'Unknown':
                        n_total_data=n_total_data+1
                        expected_time=expected_time+float(entry[0].operation_time)-float(entry[0].time_to_waypoint)
                        outcomes_count[self.state_names.index(entry[0].final_node)]+=1
                        total_outcomes_count=total_outcomes_count+1
                        del message_list[j]
                        n_unprocessed_data=n_unprocessed_data-1
                    else:
                        j=j+1
                if n_total_data==1:
                    rospy.logwarn("No data for edge between waypoints " + current_action[1] + " and " + current_action[2] + ". Assuming it to be 20 seconds. Expected time between nodes will not be correct.")
                    self.rewards[source_index][action_index]=20
                else:
                    self.rewards[source_index][action_index]=expected_time/(total_outcomes_count-1)
                    self.transitions_transversal_count[source_index][action_index]=total_outcomes_count-1
                    transition=None
                    for j in range(0,self.n_states):
                        count=outcomes_count[j]
                        if count > 0:
                            probability=float(count)/float(total_outcomes_count)
                            if transition is None:
                                transition=[[j, probability]]
                            else:
                                transition.append([j,probability])
                    if transition is not None:
                        self.transitions[source_index][action_index]=transition
        
        
    def set_initial_state_from_name(self,state_name):
        index=self.state_names.index(state_name)
        self.set_initial_state(index)



class ProductMdp(Mdp):

    def __init__(self, original_mdp,product_sta,product_lab,product_tra):
        
        self.original_mdp=original_mdp
        
        self.read_states(product_sta,product_lab) 
        self.read_actions(product_tra)
        self.read_transitions(product_tra)
        self.set_rewards_and_trans_count()
        self.set_props()
        
        
    def set_rewards_and_trans_count(self):
        
        self.rewards=[[0]*self.n_actions for i in range(self.n_states)]
        self.transitions_transversal_count=[[0]*self.n_actions for i in range(self.n_states)]
        
        for i in range(0,self.n_states):
            original_state_index=self.state_labels[i][1]
            for j in range(0,self.n_actions):
                original_action_index=self.original_mdp.actions.index(self.actions[j])
                self.rewards[i][j]=self.original_mdp.rewards[original_state_index][original_action_index]
                self.transitions_transversal_count[i][j]=self.original_mdp.transitions_transversal_count[original_state_index][original_action_index]
        
    def set_props(self):
        self.n_props=self.original_mdp.n_props
        self.props=self.original_mdp.props
        #self.props.append('ltl_goal')
        #self.n_props=self.n_props+1
        
        self.prop_map=[[False]*self.n_props for i in range(self.n_states)]
        
        for i in range(0,self.n_states):
            prop_line=self.original_mdp.prop_map[self.state_labels[i][1]]
            prop_line.append(False)
            self.prop_map[i]=prop_line
        

       
        
        
    
    def read_transitions(self,product_tra):
        f = open(product_tra, 'r')
        f.readline()
        
        self.transitions=[[False]*self.n_actions for i in range(self.n_states)]
        
        for line in f:
            line=line.split(' ')
            from_state=int(line[0])
            to_state=int(line[2])
            probability=float(line[3])
            action=self.actions.index(line[4].rstrip('\n'))
            if not self.transitions[from_state][action]:
                self.transitions[from_state][action]= [[to_state,probability]]
            else:
                self.transitions[from_state][action].append([to_state,probability])

        f.close()        
            
    
    def read_actions(self,product_tra):
        f = open(product_tra, 'r')
        f.readline()
        
        self.actions=[]
        self.n_actions=0
        
        for line in f:
            current_action=line.split(' ')[-1].rstrip('\n')
            if current_action not in self.actions:
                self.actions.append(current_action)
                self.n_actions=self.n_actions+1
                
        f.close()                
    
        
    
    def read_states(self,product_sta,product_lab):
        f = open(product_sta, 'r')
        f.readline()
        
        self.n_states=0
        self.state_labels=[]
        
        
        for line in f:
            current_state_label=line.split(':')[1]
            current_state_label=current_state_label.replace(')', '')
            current_state_label=current_state_label.replace('(', '')
            current_state_label=current_state_label.split(',')
            current_state_label[0]=int(current_state_label[0])
            current_state_label[1]=int(current_state_label[1])
            
            self.state_labels.append(current_state_label)
            
            
            
            self.n_states=self.n_states+1
            
        f.close()
        
        
        
        f = open(product_lab, 'r')
        
        line=f.readline()
        
        init_index=int(line.split('="init"')[0])
        
        target_index=line.split('="target"')[0]
        target_index=int(target_index.split(' ')[-1])
        
        
        
        
        self.goal_states=[]
        for line in f:
            line=line.split(':')
            state_index=int(line[0])
            labels=line[1].split(' ')
            del labels[0]
            n_labels=len(labels)
            for i in range(0,n_labels):
                if int(labels[i])==target_index:
                    self.goal_states.append(state_index)
                if int(labels[i])==init_index:
                    self.initial_state=state_index
        
        f.close()
        
    


    def set_policy(self,policy_file):
        self.policy=[None]*self.n_states
        f=open(policy_file,'r')
        f.readline()
        for line in f:
            line=line.split(' ')
            self.policy[int(line[0])]=line[3].strip('\n')
        print self.policy    
        f.close()    
                    
    
    def set_initial_state_from_name(self,state_name):
        state_name_prop_index=self.props.index(state_name)
        for i in range(0,self.n_states):
            if self.props_map[i][state_name_prop_index] and self.state_labels[i][0]==self.state_labels[self.initial_state][0]:
                self.set_initial_state(i)
                return
        print "set initial state of product MDP error"
    
    
    def get_new_state(self,current_state,action,final_node):
        action_index=self.actions.index(action)
        possible_next_states=self.transitions[current_state][action_index]
        n_possible_next_states=len(possible_next_states)
        final_node_prop_index=self.props.index(final_node)
        for i in range(0,n_possible_next_states):
            next_possible_state=possible_next_states[i][0]
            if self.prop_map[next_possible_state][final_node_prop_index]:
                return next_possible_state
        return -1
    
