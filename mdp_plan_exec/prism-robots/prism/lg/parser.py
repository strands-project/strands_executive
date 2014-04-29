#!/usr/bin/python

import sys


class ProductMDP(object):

    def __init__(self, original_mdp,product_sta,product_lab,product_tra):
        
        #self.n_actions=
        #self.actions=
        
        #self.n_states=
        #self.state_labels=
        #self.goal_states=
        
        #self.rewards=
        
        #self.transitions=
        
        self.read_states(product_sta,product_lab)
        
        self.read_actions(product_tra)
        
        self.read_transitions(product_tra)
        
        self.read_rewards(original_mdp)
        
        self.read_state_labels(original_mdp)
        
        
    def read_state_labels(self, original_mdp):
        f = open(original_mdp, 'r')
        
        self.props=[]
        
        self.prop_state_map=[]
        
        for line in f:
            if line.startswith('endmodule'):
                break
        
        self.n_props=0
        for line in f:
            if line.startswith('rewards'):
                break
            if line.startswith('label'):
                self.prop_state_map.append([])
                line=line.split(' ')
                self.props.append(line[1].strip('"'))
                i=3
                line_finished=False
                while not line_finished:
                    if line[i][-1]=='\n':
                        line_finished=True
                        line[i]=line[i][0:-2]
                    current_state_original=int(line[i].split('=')[1])
                    for j in range(0,self.n_states):
                        if  self.state_labels[j][1]==current_state_original:
                            self.prop_state_map[self.n_props].append(j)
                    i=i+2
                    
                    
                self.n_props=self.n_props+1    

            print(self.prop_state_map)
            print(self.props)
            
            
            
            
        
        
        
        
        
        f.close()
        
        
    
    def read_rewards(self,original_mdp):
        
        f = open(original_mdp, 'r')
         
        for line in f:
            #print(line)
            #print('rewards "time"\n')
            if line.startswith('rewards'):
                break
                
        self.rewards=[[0]*self.n_actions for i in range(self.n_states)]
        
        for line in f:
            if line.startswith('endrewards'):
                break
            line=line.split()
            if line !=[]:
                action=self.actions.index(line[0].strip('[]'))
                rest=line[1].split('=')[1].split(':')
                state=int(rest[0])
                reward=float(rest[1].rstrip(';'))
                for i in range(0,self.n_states):
                    #if i not in self.goal_states:
                        if self.state_labels[i][1]==state:
                            if self.transitions[i][action]:
                                self.rewards[i][action]=reward
                
        f.close()
        
       
        
        
    
    def read_transitions(self,product_tra):
        f = open(product_tra, 'r')
        f.readline()
        
        self.transitions=[[False]*self.n_actions for i in range(self.n_states)]
        
        for line in f:
            line=line.split(' ')
            from_state=int(line[0])
            #line[1]=int(line[1])
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
            int_line=[int(line.split(':')[0]),int(line.split(':')[1])]
            if int_line[1]==target_index:
                self.goal_states.append(int_line[0])
            if int_line[1]==init_index:
                self.initial_state=int_line[0]
                print(self.initial_state)
        
        f.close()
        
    
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
        
        f.write('label "goal" = ')
        
        goal_states_string=''
        for goal_state in self.goal_states:
            goal_states_string=goal_states_string + 's=' + str(goal_state) + ' | '
            
        f.write(goal_states_string[:-3] + ';\n')
        
        
        
        for i in range(0,self.n_props):
            string = 'label "' + self.props[i] + '" = '
            current_state_props_map=self.prop_state_map[i]
            for j in range(0,len(current_state_props_map)):
                string=string + 's=' + str(current_state_props_map[j]) + ' | '
                
            f.write(string[:-3] + ';\n')    
            
            
        f.write('\n')    
            
    
        
        f.write('rewards "time"\n')
        
        for i in range(0,self.n_states):
            for j in range(0,self.n_actions):
                if self.rewards[i][j] != 0:
                    f.write('    [' + self.actions[j] + '] s=' + str(i) + ':' + str(self.rewards[i][j]) + ';\n')
        
        f.write('endrewards\n')
        
        f.close()

            
            
if __name__ == '__main__':
    
    original_mdp=sys.argv[1]
    product_sta=sys.argv[2]
    product_lab=sys.argv[3]
    product_tra=sys.argv[4]
    output_mdp=sys.argv[5]
    
    a=ProductMDP(original_mdp,product_sta,product_lab,product_tra)
    
    a.write_prism_model(output_mdp)
    