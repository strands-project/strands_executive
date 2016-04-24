import rospy
from mongodb_store_msgs.msg import StringPair
from strands_executive_msgs.msg import MdpAction, Task

def add_string_argument(mdp_action, string_arg):
	mdp_action.arguments.append(StringPair(first=Task.STRING_TYPE, second=string_arg))

def add_int_argument(mdp_action, int_arg):
	mdp_action.arguments.append(StringPair(first=Task.INT_TYPE, second=str(int_arg)))

def add_float_argument(mdp_action, float_arg):
	mdp_action.arguments.append(StringPair(first=Task.FLOAT_TYPE, second=str(float_arg)))

def add_object_id_argument(mdp_action, oid, msg_type):
	mdp_action.arguments.append(StringPair(first=msg_type._type, second=str(oid)))

def add_time_argument(mdp_action, time_arg):
	mdp_action.arguments.append(StringPair(first=Task.TIME_TYPE, second=str(time_arg.to_sec())))

def add_duration_argument(mdp_action, duration_arg):
	mdp_action.arguments.append(StringPair(first=Task.DURATION_TYPE, second=str(duration_arg.to_sec())))

def add_bool_argument(mdp_action, bool_arg):
	mdp_action.arguments.append(StringPair(first=Task.BOOL_TYPE, second=str(bool_arg)))

