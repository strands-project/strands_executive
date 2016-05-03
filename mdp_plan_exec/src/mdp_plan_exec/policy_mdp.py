import rospy
from random import choice
from copy import deepcopy

from mdp import Mdp


        

class PolicyMdp(Mdp):
    def __init__(self, original_mdp, product_sta, product_lab, policy_tra, probs_vect, progs_vect, exp_times_vect):
        Mdp.__init__(self)
        self.original_mdp=original_mdp
        self.n_state_vars=original_mdp.n_state_vars+1
        self.state_vars=list(original_mdp.state_vars)
        self.state_vars.append("_da")
        self.state_vars_range=dict(original_mdp.state_vars_range) #ranges for the state vars
        self.initial_state=dict(original_mdp.initial_state) #dict indexed by the state vars
        self.n_props=original_mdp.n_props #number of propositional labels
        self.props=list(original_mdp.props)
        self.props_def=dict(original_mdp.props_def) #dict of MdpPropDef instances. keys are the propositional labels names
        self.n_actions=original_mdp.n_actions #number of actions
        self.actions=list(original_mdp.actions) #list of action names
        self.transitions=[] #list of MdpTransitionDef instances: won't be filled for the policy, as we will work with the flat representations
        self.reward_names=list(original_mdp.reward_names)
        
        #read sta product file to get flat state descriptions and number of dfa states
        self.n_aut_states=0
        self.n_flat_states=0
        self.flat_state_defs={}
        self.read_prod_state_file(product_sta)
        self.state_vars_range["_da"]=[0, self.n_aut_states]
        
        #read lab product file to get initial and accepting states
        self.initial_flat_state=-1
        self.acc_flat_states=set()
        self.set_init_and_acc_states(product_lab)

        
        self.flat_state_policy={} #self.flat_state_policy[flat_state]=action to execute in flat_state
        self.flat_state_sucs={} #self.flat_state_sucs[flat_state]=list of possible flat state successors, e.g., [20,25]
        self.flat_state_suc_probs={} #self.flat_state_probs[flat_state]=list of probs associated to the corresponding flat_state_sucs, e.g., [0.7,0.3]
        self.transitions=[] #not being set for efficiency. The flat representations above are easier to build and to use for execution. only needed for exporting of the policy
        self.set_policy_flat(policy_tra)
        
        self.guarantees_probs=self.read_vect(probs_vect)
        self.guarantees_progs=self.read_vect(progs_vect)
        self.guarantees_times=self.read_vect(exp_times_vect)
        
        #print "AHAH", self.guarantees_probs
    
    
    ######Parsing methods#######
    def read_vect(self, file_name):
        res={}
        i=0
        f=open(file_name, 'r')
        for line in f:
            res[i]=float(line)
            i+=1
        f.close()
        return res
        
    
    def set_policy_flat(self,file_name):
        f = open(file_name , 'r')

        f.readline()        
        for line in f:
            line_array = line.split(' ')
            line_array[-1] = line_array[-1].strip('\n')
            source = int(line_array[0])
            target = int(line_array[1])
            prob = float(line_array[2])
            action = line_array[3]
            if self.flat_state_policy.has_key(source):
                self.flat_state_sucs[source].append(target)
                self.flat_state_suc_probs[source].append(prob)
            else:
                self.flat_state_policy[source] = action
                self.flat_state_sucs[source]=[target]
                self.flat_state_suc_probs[source]=[prob]
        f.close()
        
        
    
    def set_init_and_acc_states(self, labels_file):
        f=open(labels_file, 'r')
        line=f.readline()        
        label_names=line.split(' ')
        for label in label_names:
            label_pair=label.split('=')
            if label_pair[1].strip('\n')=='"init"':
                init_index=int(label_pair[0])
            if label_pair[1].strip('\n')=='"target"':
                acc_index=int(label_pair[0])

        for line in f:
            line=line.split(':')
            state_index=int(line[0])
            labels=line[1].split(' ')
            del labels[0]
            for label in labels:
                if int(label)==init_index:
                    self.initial_state=dict(self.flat_state_defs[state_index])
                    self.initial_flat_state=init_index
                if int(label)==acc_index:
                    self.acc_flat_states.add(int(label))
        f.close()
        
    
    def read_prod_state_file(self, states_file):
        f = open(states_file, 'r')        
        variables = f.readline()
        variables = variables.split(',')
        variables[0] = variables[0].strip('(')
        variables[-1] = variables[-1].strip(')\n')
       
        for line in f:
            states_string = line.split(':')
            state_id=int(states_string[0])
            flat_state_list = states_string[1].split(',')
            flat_state_list[0] = flat_state_list[0].strip('(')
            flat_state_list[-1] = flat_state_list[-1].strip(')\n')
            flat_state_dict={}
            for (var_name, value) in zip(variables, flat_state_list):
                flat_state_dict[var_name] = int(value)
                if var_name == '_da':
                    self.n_aut_states=max(self.n_aut_states, int(value))
            self.flat_state_defs[state_id]=flat_state_dict
            self.n_flat_states+=1
        f.close()
        
    
    
    def set_n_aut_states(self, dfa_file_name):
        f=open(dfa_file_name, 'r')
        line=f.readline()
        line=line.split(' ')        
        #read states
        self.n_aut_states=int(line[0])
        self.aut_initial_state=int(line[3].replace('),',''))
    ######Parsing methods#######


    def get_guarantees_at_flat_state(self, flat_state):
        return (self.guarantees_probs[flat_state], 
                self.guarantees_progs[flat_state], 
                rospy.Duration(self.guarantees_times[flat_state]))


    def simulate_random(self):
        current_flat_state = self.initial_flat_state
        while True:
            if self.flat_state_policy.has_key(current_flat_state):
                action = self.flat_state_policy[current_flat_state]
                print action
                current_flat_state = choice(self.flat_state_sucs[current_flat_state])
            else:
                print("FINISHED")
                return
            

   
    
