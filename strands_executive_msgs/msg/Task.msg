

# Constant to represent no task 
uint64 NO_TASK=0
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


# Some fixed constants to give examples of priorities
uint64 LOW_PRIORITY=100
uint64 MEDIUM_PRIORITY=200
uint64 HIGH_PRIORITY=300


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

# The expected duration of the task
duration expected_duration

# The scheduler execution time
time execution_time

# Priority of a task - higher number = higher priority
uint64 priority

# Additional arguments to the action server. These are interpreted in two ways, either "","string" is interpreted as a string or "ros message type", "datacentre id" is a reference to the ObjectID of an entry in the datacentre obtained from the result of MongoInsertMsg.srv. 
mongodb_store_msgs/StringPair[] arguments
