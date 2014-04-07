# STRANDS Executive


Executive control code for STRANDS robots. The basic unit of behaviour is a *task* as defined by `strands_executive_msgs/Task`. To get the robot to execute a task an appropriate instance of the `Task` message must be sent to the task executor framework. Currently only the `fifo_task_executor.py` exists, which executes tasks in a FIFO manner, but later on a scheduler will be added.


## Creating a Task

Most task instances will contain both the name of a [topological map node](https://github.com/strands-project/strands_navigation/tree/hydro-devel/topological_navigation) where the task should be executed, plus the name of a [SimpleActionServer](http://wiki.ros.org/actionlib) to be called at the node and its associated arguments. Tasks must contain one of these things, but not necessarily both (just a node name will just take the robot there, just an action will execute the action without moving the robot).

To create a task, first create an instance of the `Task` message type. Examples are given in Python, as the helper functions currently only exist for Python, but C++ is also possible (and C++ helpers will be added soon).

```python
from strands_executive_msgs.msg import Task
task = Task()
```

Then you can set the node id for where the task will be executed (or you can do this inline):

```python
task.node_id = 'WayPoint1'
```

To add the name of the action server, do:

```python
task.action = 'do_dishes'
```

Where 'do_dishes' is replaced by the first argument you would give to the `actionlib.SimpleActionClient` constructor.

If your actionlib goal type needs arguments, you must then add them to the task **in the order they are used in your goal type constructor**. You can either add plain string arguments (which are stored in the task itself) or ROS message instances (which are stored in the [ros_datacentre](https://github.com/strands-project/ros_datacentre)). For example, for the following action which is available under [task_executor/action/TestExecution.action](https://github.com/strands-project/strands_executive/blob/hydro-devel/task_executor/action/TestExecution.action)

```
# something to print
string some_goal_string
# something to test typing
geometry_msgs/Pose test_pose
---
---
# feedback message
float32 percent_complete
```

You need to supply a string argument followed by a pose. To add the string, do the following

```python
from strands_executive_msgs import task_utils
task_utils.add_string_argument(task, 'my string argumment goes here')
```

For the pose, this must be added to the ros_datacentre message store and then the `ObjectID` of the pose is used to communicate its location. This is done as follows

```python
p = Pose()
object_id = msg_store.insert(p)
task_utils.add_object_id_argument(task, object_id, Pose)
```

Finally the task can be registered with the task executor and started:

```python
add_task_srv_name = '/task_executor/add_task'
set_exe_stat_srv_name = '/task_executor/set_execution_status'
rospy.wait_for_service(add_task_srv_name)
rospy.wait_for_service(set_exe_stat_srv_name)
add_task_srv = rospy.ServiceProxy(add_task_srv_name, AddTask)
set_execution_status = rospy.ServiceProxy(set_exe_stat_srv_name, SetExecutionStatus)
    
try:
	# add task to the execution framework
    task_id = add_task_srv(task)
    # make sure execution is running -- this only needs to be done onece      
    set_execution_status(True)
except rospy.ServiceException, e: 
	print "Service call failed: %s"%e		
```



