# Message for logging execution of co-safe LTL taskon the MDP. Contains predicted and real values

# The MDP definition and goal for this task
strands_executive_msgs/ExecutePolicyActionGoal mdp_goal

# The sequence of feedbacks obtained from the MDP action server when executing this action
strands_executive_msgs/ExecutePolicyActionFeedback[] feedbacks

# The sequence of observed execution durations for each action represented in the feedbacks array (obtained from the timestamps of the feedbacks)
duration[] durations