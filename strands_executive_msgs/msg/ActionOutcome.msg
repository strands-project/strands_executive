#message representing an action outcome in an MDP
float64 probability #prob of this outcome happening
string conds #string representing the state variables update associated to this outcome
float64[] duration_probs #list of probabilities associated to duration clusters for this outcome
float64[] durations #list of duration clusters representatives. same lenght as duration_probs
actionlib_msgs/GoalStatus[] status #list of goal status outcomes associated to this outcome
string result #string representing the outcome of the action server that is associated to this outcome