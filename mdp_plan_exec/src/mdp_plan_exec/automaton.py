import rospy

class Automaton(object):
    def __init__(self, product_aut):
        self.n_states=0
        self.initial_state=0
        self.accepting_states=[]
        self.sink_states=[]
        self.transitions=[]
        self.inverse_transitions=[]
        self.distances_to_goal=[]
        
        f=open(product_aut, 'r')
        line=f.readline()
        line=line.split(' ')
        
        #read states
        self.n_states=int(line[0])
        self.transitions=[[] for i in range(0,self.n_states)]
        self.inverse_transitions=[[] for i in range(0,self.n_states)]
        self.initial_state=int(line[3].replace('),',''))
        if line[-3]=='Finite':
            accepting_states_string=line[-1].replace('{','').replace('}','').replace('\n','').split(',')
            if len(accepting_states_string)!=1:
                rospy.logerr("More than one accepting state. The automaton construction will not work.")
            self.accepting_states=[int(accepting_states_string[0])]
        else:
            rospy.logwarn("Not a DFA, check formula")
            if line[-4] != '1':
                rospy.logerr("More than one acceptance pair. The automaton construction will not work.")
            accepting_states_string=line[-1].replace('(','').replace(')','').replace('{','').replace('}','').replace('\n','').split(',')
            print accepting_states_string
            if len(accepting_states_string)!=2:
                rospy.logerr("More than one accepting state. The automaton construction will not work.")
            self.accepting_states=[int(accepting_states_string[1])]
        
        #iterate over useless labels defs
        i=4
        while True:
            if line[i][-2:] == '):':
                break            
            i=i+1
        i=i+1
        
        #read transitions
        reading_trans=True
        while reading_trans:
            trans_string=line[i].split('-')
            source=int(trans_string[0])
            if len(trans_string)==3:
                target_string=trans_string[2][1:]
            elif len(trans_string)==2:
                i=i+1
                while True:
                    trans_string=line[i].split('-')
                    if len(trans_string)==2:
                        break
                    i=i+1
                target_string=trans_string[1][1:]               
            else:
                rospy.logerr("BUG ON AUTOMATON TRANS PARSING!")
            if target_string[-1]==';':
                target_string=target_string[:-1]
                reading_trans=False
            target=int(target_string)
            try:
                target_index=[a for [a,b] in self.transitions[source]].index(target)
                inv_source_index=[a for [a,b] in self.inverse_transitions[target]].index(source)
                self.transitions[source][target_index][1]+=1.0
                self.inverse_transitions[target][inv_source_index][1]+=1.0
            except ValueError:                
                self.transitions[source].append([target,1.0])
                self.inverse_transitions[target].append([source, 1.0])
            i=i+1
        #calculate distances to goal, wighted by number of transitions between states
        self.distances_to_goal = [self.n_states for i in range(0,self.n_states)]
        queue=list(self.accepting_states)
        for state in self.accepting_states:
            self.distances_to_goal[state]=0
        while queue!=[]:
            current_state=queue.pop(0)
            for state in self.inverse_transitions[current_state]:
                distance=self.distances_to_goal[current_state]+(1.0/state[1])
                if self.distances_to_goal[state[0]]>distance:
                    self.distances_to_goal[state[0]]=distance
                    queue.append(state[0])
        rospy.loginfo("Automaton states distances to goal: " + str(self.distances_to_goal))
        
        #get sink states
        for i in range(0,self.n_states):
            if self.distances_to_goal[i] == self.n_states:
                self.sink_states.append(i)
                                        
                    

