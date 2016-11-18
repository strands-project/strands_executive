# message representing an action outcome in an MDP

# Constant for float type argument
string FLOAT_TYPE="____float____"
# Constant for int type argument
string INT_TYPE="____int____"
# Constant for ros time type argument
string TIME_TYPE="____time____"
# Constant for ros duration type argument
string DURATION_TYPE="____duration____"
# Constant for bool type
string BOOL_TYPE="____bool____"
# Constant for string type
string STRING_TYPE="____str____"


# prob of this outcome happening
float64 probability 

# waypoint position after this outcome happening
string waypoint 

# each element of the form (var_key, val). if this is the action outcome, the executor updates the current MDP state dict such that current_state[var_key]=val, for each element of the list. 
strands_executive_msgs/StringIntPair[] post_conds  

# list of probabilities associated to duration clusters for this outcome
float64[] duration_probs 

# list of duration clusters representatives. same lenght as duration_probs list, as elements in the 2 lists with the same index are related
float64[] durations 

# list of goal status, as defined in actionlib_msgs/GoalStatus.msg outcomes associated to this outcome. After the action is executed, we check if its final GoalStatus is in this list.
int8[] status 

# (attribute, type, value). If the check of the status above is True, then we go through each element of the list and check if result.attribute==value. if so, then this is the outcome and we update the state"
strands_executive_msgs/StringTriple[] result 