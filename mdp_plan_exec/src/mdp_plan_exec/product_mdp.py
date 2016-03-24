import rospy
from mdp import Mdp, MdpTransitionDef, MdpPropDef
from automaton import Automaton



class ProductMdp(Mdp):
    def __init__(self, original_mdp,product_sta,product_lab,product_tra, product_aut):
        Mdp.__init__(self)
        
        self.original_mdp=original_mdp
        #check if its already a product mdp
        if isinstance(original_mdp, ProductMdp):
            self.n_goals=original_mdp.n_goals+1
        else:
            self.n_goals=1
        self.n_state_vars=original_mdp.n_state_vars+1
        self.state_vars=list(original_mdp.state_vars)
        self.dra_state_name="dra_state"+str(self.n_goals)
        self.state_vars.append(self.dra_state_name)
        self.state_vars_range=dict(original_mdp.state_vars_range) #ranges for the state vars
        self.initial_state=dict(original_mdp.initial_state) #dict indexed by the state vars
        self.n_props=original_mdp.n_props+1 #number of propositional labels
        self.props=list(original_mdp.props)
        self.accepting_prop_name="dra_acc_state"+str(self.n_goals)
        self.props.append(self.accepting_prop_name)#list of propositional label names
        self.props_def=dict(original_mdp.props_def) #dict of MdpPropDef instances. keys are the propositional labels names
        self.n_actions=original_mdp.n_actions #number of actions
        self.actions=list(original_mdp.actions) #list of action names
        self.transitions=[] #list of MdpTransitionDef instances
        self.transitions_sources_flat=[]
        self.reward_names=list(original_mdp.reward_names)
        self.ltl_reward_struct_name="goal" + str(self.n_goals) + "_rew"
        self.reward_names.append(self.ltl_reward_struct_name)
        self.current_policy=[]
        self.n_total_states=0

        #read sta product file       
        f = open(product_sta, 'r')
        line=f.readline()
        line=line.replace(')', '')
        line=line.replace('(', '')
        line=line.replace('\n', '')
        product_labels=line.split(',')
        for i in range(0, len(product_labels)):
            if product_labels[i]=='_dra' or product_labels[i]=='_da':
                product_labels[i]=self.dra_state_name
                break
        
        self.product_state_defs=[]
        for line in f:
            line=line.replace(')', '')
            line=line.replace('(', '')
            line=line.replace('\n', '')
            line=line.split(':')[1]
            line=line.split(',')
            state={}
            for i in range(0, len(line)):
                state[product_labels[i]]=int(line[i])
            self.product_state_defs.append(state)
            self.n_total_states=self.n_total_states+1
        rospy.loginfo("The product has a total of " + str(self.n_total_states) + " states.")    
        f.close()
        
        #read lab file to find initial state
        f=open(product_lab, 'r')
        line=f.readline()        
        init_index=int(line.split('="init"')[0])               
        for line in f:
            line=line.split(':')
            state_index=int(line[0])
            labels=line[1].split(' ')
            del labels[0]
            for label in labels:
                if int(label)==init_index:
                    self.initial_state=dict(self.product_state_defs[state_index])
                    break
        f.close()
        
        #read aut product file
        self.automaton=Automaton(product_aut)
        self.state_vars_range[self.dra_state_name]=(0,self.automaton.n_states-1)
        accept_def=MdpPropDef(name=self.accepting_prop_name)
        conds={}

        for state in self.automaton.accepting_states:
            conds[self.dra_state_name]=state
        accept_def.conds=conds
        self.props_def[self.accepting_prop_name]=accept_def

        #read tra product file - to define mdp
        self.inverse_mdp_graph=[[] for i in range(0,self.n_total_states)]
        self.possible_reward_states=[]
        f = open(product_tra, 'r')
        f.readline()

        from_state=-1
        action_number=-1
        for line in f:
            line=line.replace('\n','')
            line=line.split(' ')
            old_trans=(from_state==int(line[0]) and action_number==int(line[1]))
            from_state=int(line[0])
            from_state_def=self.product_state_defs[from_state]
            action_number=int(line[1])
            to_state=int(line[2])
            to_state_def=self.product_state_defs[to_state]
            probability=float(line[3].replace(',','.'))
            action_name=line[4]
            if action_name=='':
                continue
            if from_state not in self.inverse_mdp_graph[to_state]:
                self.inverse_mdp_graph[to_state].append(from_state)
            if old_trans:
                self.transitions[-1].prob_post_conds.append((probability, dict(to_state_def)))
            else:
                original_transition=self.get_original_transition(action_name,from_state_def)
                self.transitions_sources_flat.append(from_state)
                self.transitions.append(MdpTransitionDef(action_name=action_name,
                                                  pre_conds=dict(from_state_def),
                                                  prob_post_conds=[(probability, dict(to_state_def))],
                                                  rewards=dict(original_transition.rewards)))
                self.transitions[-1].rewards[self.ltl_reward_struct_name]=0

            self.transitions[-1].rewards[self.ltl_reward_struct_name]=self.transitions[-1].rewards[self.ltl_reward_struct_name]+probability*(self.automaton.distances_to_goal[from_state_def[self.dra_state_name]]-self.automaton.distances_to_goal[to_state_def[self.dra_state_name]])
            if from_state not in self.possible_reward_states and self.transitions[-1].rewards[self.ltl_reward_struct_name]>0:
                self.possible_reward_states.append(from_state)
        f.close()

        queue=list(self.possible_reward_states)
        while queue!=[]:
            current=queue.pop(0)
            for predecessor in self.inverse_mdp_graph[current]:
                if predecessor not in self.possible_reward_states:
                    self.possible_reward_states.append(predecessor)
                    queue.append(predecessor)
                    
      
        n_deleted=0
        for i in range(0,len(self.transitions_sources_flat)):
            state=self.transitions_sources_flat[i]
            if state not in self.possible_reward_states:
                del self.transitions[i-n_deleted]
                n_deleted+=1
                #self.transitions[i].rewards["time"]=0
                    

    def get_original_transition(self, action_name, from_state_def):
        for transition in self.original_mdp.transitions:
            if transition.action_name == action_name and self.check_cond_sat(transition.pre_conds, from_state_def):
                return transition
        rospy.logerr('TRANS NOT FOUND'+str(from_state_def))

    def set_policy(self,policy_file):
        f=open(policy_file,'r')
        f.readline()
        self.policy=[None]*self.n_total_states
        for line in f:
            line=line.split(' ')
            self.policy[int(line[0])]=line[3].strip('\n')
        rospy.loginfo("Policy: " + str(self.policy))
        f.close()    
                    
    
    #def set_initial_state_from_name(self,state_name):
        #state_name_prop_index=self.props.index(state_name)
        #for i in range(0,self.n_states):
            #if self.props_map[i][state_name_prop_index] and self.state_labels[i][0]==self.state_labels[self.initial_state][0]:
                #self.set_initial_state(i)
                #return
        #print "set initial state of product MDP error"
    
    
    #def get_new_state(self,current_state,action,final_node):
        #action_index=self.actions.index(action)
        #possible_next_states=self.transitions[current_state][action_index]
        #n_possible_next_states=len(possible_next_states)
        #final_node_prop_index=self.props.index(final_node)
        #for i in range(0,n_possible_next_states):
            #next_possible_state=possible_next_states[i][0]
            #if self.prop_map[next_possible_state][final_node_prop_index]:
                #self.publish_current_policy_mode(next_possible_state)
                #return next_possible_state
        #return None
        
    #def publish_current_policy_mode(self, current_state):
        #sources = []
        #targets = []
        #current_mode = self.state_labels[current_state][0]
        #policy_msg = NavRoute()
        #print current_mode
        #for i in range(0,self.n_states):
            #current_action = self.policy[i]
            #if current_action is not None and self.state_labels[i][0] == current_mode:
                #action_split = current_action.split('_')
                #source = action_split[1]
                #target = action_split[2]
                #policy_msg.source.append(source)
                #policy_msg.target.append(target)
                
        #self.policy_publisher.publish(policy_msg)
                
        
        
   
        
        
        
    
