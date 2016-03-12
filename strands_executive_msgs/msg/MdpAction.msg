#message representing an action an MDP

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

string name #action name in the MDP
string action_server #action server to be called
mongodb_store_msgs/StringPair[] arguments # Arguments to the action server. These are interpreted in two ways, either ("","string") is interpreted as a string or ("ros message type", "datacentre id") is a reference to the ObjectID of an entry in the datacentre obtained from the result of MongoInsertMsg.srv. 
string[] waypoints #list of waypoints where action can be executed
strands_executive_msgs/StringIntPair[] pre_conds #string representing other valuations of state variables that need to hold for the action to be active USE diagnostic_msgs/KeyValue?
strands_executive_msgs/MdpActionOutcome[] outcomes #list of possible outcomes. sum of outcome.probability should be 1

