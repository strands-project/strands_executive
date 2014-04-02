
# This holds the id of the task. This will be assigned by the framework, and doesn't need to be provided.

uint64 task_id

# The topological map node where this task should start
string node_id

# The action server to be called to trigger the task
string action

# Additional arguments to the action server. These are interpreted in two ways, either "","string" is interpreted as a string or "ros message type", "datacentre id" is a reference to the ObjectID of an entry in the datacentre obtained from the result of MongoInsertMsg.srv. 
ros_datacentre_msgs/StringPair[] arguments
