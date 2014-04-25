
# Constant to represent no task 
uint64 NO_TASK=0
# Constant for float type argument
string FLOAT_TYPE="____float____"
# Constant for int type argument
string INT_TYPE="____int____"


# This holds the id of the task. This will be assigned by the framework, and doesn't need to be provided.
uint64 task_id

# The topological map node where this task should start
string start_node_id

# The topological map node where this task should end
string end_node_id

# The action server to be called to trigger the task
string action

# The task cannot start before this time
time start_after

# The task be completed after this time
time end_before

# The maximum duration of the task
duration max_duration

# The scheduler execution time
time execution_time

# Additional arguments to the action server. These are interpreted in two ways, either "","string" is interpreted as a string or "ros message type", "datacentre id" is a reference to the ObjectID of an entry in the datacentre obtained from the result of MongoInsertMsg.srv. 
ros_datacentre_msgs/StringPair[] arguments
