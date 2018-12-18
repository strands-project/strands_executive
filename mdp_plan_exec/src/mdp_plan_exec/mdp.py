class MdpTransitionDef(object):
    def __init__(self, 
                action_name=None,
                pre_conds=None,
                prob_post_conds=None,
                rewards=None):
        self.action_name=action_name
        self.pre_conds=pre_conds
        self.prob_post_conds=prob_post_conds
        self.rewards=rewards
        
class MdpPropDef(object):
    def __init__(self,
                name=None,
                conds=None):
        self.name=name
        self.conds=conds
    
    
class Mdp(object):
    def __init__(self):
        #list of attributes of an MDP object
        self.n_state_vars=0 #numer of variables used to defined the mdp state
        self.state_vars=[] #names of the different variables used to define the state
        self.state_vars_range={} #dict with ranges (min,max) for the state vars
        self.initial_state={} #dict indexed by the state vars
        self.n_props=0 #number of propositional labels
        self.props=[] #list of propositional label names
        self.props_def={} #dict of MdpPropDef instances. keys are the propositional labels names
        self.n_actions=0 #number of actions
        self.actions=[] #list of action names
        self.transitions=[] #list of MdpTransitionDef instances
        self.current_policy=[]
        self.reward_names=[]

    def check_cond_sat(self, cond, state):
        for var in cond:
            if cond[var]!=state[var]:
                return False
        return True

    
    def cond_to_prism_string(self, cond, is_post_cond=False):
        conds_string=''
        for key, value in cond.iteritems():
            if is_post_cond:
                key=key+"'"
            conds_string=conds_string + '(' + key + '=' + str(value) + ') & '
        return conds_string[:-3]
    
       
    def write_prism_model(self,file_name, set_initial_state=True):
        f=open(file_name,'w')
        f.write('mdp\n \n')
        f.write('module M \n \n')
        
        for state_var in self.state_vars:
            state_var_range=self.state_vars_range[state_var]
            if set_initial_state:
                f.write(state_var + ':[' + str(state_var_range[0]) + '..' + str(state_var_range[1]) + '] init ' + str(self.initial_state[state_var]) + ';\n')
            else:
                f.write(state_var + ':[' + str(state_var_range[0]) + '..' + str(state_var_range[1]) + ']' + ';\n') 
        f.write('\n')
        
        for transition in self.transitions:
            prob_post_conds_string=''
            for pair in transition.prob_post_conds:
                prob_post_conds_string=prob_post_conds_string + str(pair[0]) + ':' + self.cond_to_prism_string(pair[1], True) + ' + '
            f.write('[' + transition.action_name + '] ' + self.cond_to_prism_string(transition.pre_conds,False) + ' -> ' + prob_post_conds_string[:-3] + ';\n')    
        f.write('\nendmodule\n\n')
        
        for name, prop in self.props_def.iteritems():            
            f.write('label "' + name + '" = ' + self.cond_to_prism_string(prop.conds) + ';\n')
        
        for reward in self.reward_names:
            f.write('\n')
            f.write('rewards "'+ reward + '"\n')
            for transition in self.transitions:
                if reward in transition.rewards and transition.rewards[reward]!=0:
                    f.write('   [' + transition.action_name + '] ' + self.cond_to_prism_string(transition.pre_conds) + ':' + str(transition.rewards[reward]) + ';\n')    
            f.write('endrewards\n')
        
        if not set_initial_state:
            f.write('\n init true endinit')
        
        f.close()
        

        
    #def set_initial_state(self,initial_state):
        #self.initial_state=initial_state
        
    #def get_expected_edge_transversal_time(self,state_index,action_name):
        #action_index=self.actions.index(action_name)
        #return self.rewards[state_index][action_index]
        
    #def get_total_transversals(self,state_index,action_name):
        #action_index=self.actions.index(action_name)
        #return self.transitions_transversal_count[state_index][action_index]
        
