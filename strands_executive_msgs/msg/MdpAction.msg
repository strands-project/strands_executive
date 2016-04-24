#message representing an action an MDP

#action name in the MDP
string name 

#action server to be called
string action_server 

# Arguments to the action server. These are interpreted in two ways, either ("","string") is interpreted as a string or ("ros message type", "datacentre id") is a reference to the ObjectID of an entry in the datacentre obtained from the result of MongoInsertMsg.srv. 
mongodb_store_msgs/StringPair[] arguments 

#list of waypoints where action can be executed
string[] waypoints 

#string representing other valuations of state variables that need to hold for the action to be active USE diagnostic_msgs/KeyValue?
strands_executive_msgs/StringIntPair[] pre_conds 

#list of possible outcomes. sum of outcome.probability should be 1
strands_executive_msgs/MdpActionOutcome[] outcomes 

