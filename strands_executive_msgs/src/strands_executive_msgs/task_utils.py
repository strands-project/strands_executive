import rospy
from ros_datacentre_msgs.msg import StringPair
from strands_executive_msgs.msg import Task

def add_string_argument(task, string_arg):
	task.arguments.append(StringPair(second=string_arg))

def add_int_argument(task, int_arg):
	task.arguments.append(StringPair(first=Task.INT_TYPE, second=str(int_arg)))

def add_float_argument(task, float_arg):
	task.arguments.append(StringPair(first=Task.FLOAT_TYPE, second=str(float_arg)))


def add_object_id_argument(task, oid, msg_type):
	task.arguments.append(StringPair(first=msg_type._type, second=str(oid)))

