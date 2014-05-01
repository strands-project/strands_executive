# STRANDS Executive


Executive control code for STRANDS robots. The basic unit of behaviour is a *task* as defined by `strands_executive_msgs/Task`. To get the robot to execute a task an appropriate instance of the `Task` message must be sent to the task executor framework. Currently only the `fifo_task_executor.py` exists, which executes tasks in a FIFO manner, but later on a scheduler will be added.

## Dependencies

Notes, before you can run any of the scheduling stuff, you must run have the datacentre running, e.g.

```bash
roslaunch ros_datacentre datacentre.launch
```

and you need to be offering a 'topological_navigation', GotoNodeAction action. If you're not running the full topological navigation system, you can run

```bash
rosrun task_executor test_task_action.py
```

which will fake this.

## Running scheduled patrols

To test the executive framework you can try running the robot around the topological map. To do this, first get your basic 2D navigation setup running (in simulation or reality). Next, build a [topological map](https://github.com/strands-project/strands_navigation/tree/hydro-devel/topological_navigation). Then (assuming you have no special transitions in your map), run the [monitored navigation](https://github.com/strands-project/strands_navigation/tree/hydro-devel/monitored_navigation) plus the topological navigation:

```bash
roslaunch monitored_navigation monitored_nav.launch
roslaunch topological_navigation topological_navigation.launch map:=<topological_map_name> node_by_node:=false
```

Then you can start the executive framework:

```bash
roslaunch task_executor task-scheduler.launch
```

With this up and running you can start the robot running continuous patrols using:

```bash
rorun task_executor continuous_patrolling.py
```

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

You should also set the expected duration of the task. This will be used by the scheduler and also by the execution framework to determine whether it things your task may be misbehaving. The duration is a `rospy.Duration` instance and is defined in seconds. 

```python
dishes_duration = 60 * 60
task.max_duration = rospy.Duration(dishes_duration)
```

If your actionlib goal type needs arguments, you must then add them to the task **in the order they are used in your goal type constructor**. You can either add plain string arguments (which are stored in the task itself) or ROS message instances (which are stored in the [ros_datacentre](https://github.com/strands-project/ros_datacentre)). For example, for the following action which is available under [task_executor/action/TestExecution.action](https://github.com/strands-project/strands_executive/blob/hydro-devel/task_executor/action/TestExecution.action)

```
# something to print
string some_goal_string
# something to test typing
geometry_msgs/Pose test_pose
# something for numbers
int32 an_int
float32 a_float
---
---
# feedback message
float32 percent_complete
```

You need to supply a string argument followed by a pose, then an int then a float. To add the string, do the following

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

Ints and floats can be added as follows

```python
task_utils.add_int_argument(task, 24)
task_utils.add_float_argument(task, 63.678)
```

### Registering a Task

A single task can be registered with the task executor and started:

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

## Creating a Routine

The scenario use case for task execution is that the robot has a *daily routine* which is a list of tasks which it carries out every day. This can be created with the `task_routine.DailyRoutine` object which is configured with start and end times for the robot's daily activities:

```python
	# some useful times
    localtz = tzlocal()
    # the time the robot will be active
    start = time(8,30, tzinfo=localtz)
    end = time(17,00, tzinfo=localtz)
    midday = time(12,00, tzinfo=localtz)

    morning = (start, midday)
    afternoon = (midday, end)

    routine = task_routine.DailyRoutine(start, end)
 ```

Tasks are then added using the `repeat_every*` methods. These take the given task and store it such that it can be correctly instantiated with start and end times every day:

```python
	# do this task every day
    routine.repeat_every_day(task)
    # and every two hours during the day
    routine.repeat_every_hour(task, hours=2)
    # once in the morning
    routine.repeat_every(task, *morning)
    # and twice in the afternoon
    routine.repeat_every(task, *afternoon, times=2)

```


The `DailyRoutine` is a static thing (just a bunch of convenience functions really). The routine tasks are passed to the `DailyRoutineRunner` which manages the creation of specific task instances according to the schedule, and then sends them to the scheduler before they need to be added to teh schedule. 


```python

	# this uses the newer AddTasks service which excepts tasks as a batch
	add_tasks_srv_name = '/task_executor/add_tasks'
	add_tasks_srv = rospy.ServiceProxy(add_tasks_srv_name, AddTasks)


	# create the object which will talk to the scheduler
    runner = task_routine.DailyRoutineRunner(start, end, add_tasks_srv)
    # pass the routine tasks on to the runner which handles the daily instantiation of actual tasks
    runner.add_tasks(routine.get_routine_tasks())

    # Set the task executor running (if it's not already)
    set_execution_status(True)
```    