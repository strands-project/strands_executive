# The STRANDS Executive Framework

An executive framework for a mobile robot. The basic unit of behaviour for the framework is a *task* as defined by `strands_executive_msgs/Task`. The framework triggers and manages the execution of tasks plus navigation between them. The framework includes a task scheduler to optimise the execution time of a list of times. 


## Installation

### Binary

If you are using Ubuntu, the easiest way to install the executive framework is to [add the LCAS apt releases repository](https://github.com/lcas/rosdistro/wiki). You can then run `sudo apt-get install ros-indigo-task-executor` or `ros-kinetic-task-executor`. This will install the framework and it's dependencies alongside your existing ROS install under `/opt/ros/indigo`. Note that due to java versions, the indigo version is running an older version of prism and many of the components, **we therefore recommend using kinetic if possible**. If you want to run the latest version of the framework under indigo then you need to install from source and install a Java 8 package.

### Source

To compile from source you should clone this repository into your catkin workspace and compile as normal. For dependencies you will also need (at least) the following repsositories: [strands_navigation](https://github.com/strands-project/strands_navigation) and [mongodb_store](https://github.com/strands-project/mongodb_store), and Java 8 or greater (which is the default on 16.04)/ Source installs have been tested on Ubuntu 14.04 and 16.04.

## Runtime Dependencies

For the executive framework to function correctly, you must have the mongodb_store nodes running. These are used by the framework to store tasks with arbitrary arguments.

```bash
roslaunch mongodb_store mongodb_store.launch
```
or with path specifying, where should the db is stored:

```bash
roslaunch mongodb_store mongodb_store.launch db_path:=/...
```

Currently the framework abstracts over navigation actions using [the STRANDS topological navigation framework](https://github.com/strands-project/strands_navigation/tree/hydro-devel/topological_navigation). Therefore you must have this framework running. For testing, or if you're not running the full topological navigation system, you can run a simulple simulated topological system with the following command:

```bash
roslaunch topological_utils dummy_topological_navigation.launch
```

This produces a map with 9 nodes: `ChargingPoint` in the centre, with `v_-2`, `v_-1` to `v_2` running vertically and `h_-2` to `h_2` running horizontally, joining `ChargingPoint` in the middle.

## Running the executive framework

To start the executive framework, launch the following launch file.

```bash
roslaunch task_executor task-scheduler-top.launch
```

This launches using topolgical navigation for moving the robot. If you wish to use the MDP execution (which has additional runtime dependencies) replace `top` with `mdp` in the launch file name.

## Running scheduled patrols

To test the executive framework you can try running the robot around the topological map.

Start the executive framework:

```bash
roslaunch task_executor task-scheduler.launch
```

With this up and running you can start the robot running continuous patrols using:

```bash
rorun task_executor continuous_patrolling.py
```

If this runs successfully, then your basic systems is up and running safely.


## Tasks

This executive framework, schedules and manages the execution of *tasks*. A task maps directly to the execution of an [actionlib action server](http://wiki.ros.org/actionlib), allowing you to resuse or integrate your desired robot behaviours in a widely used ROS framework.

Most task instances will contain both the name of a [topological map node](https://github.com/strands-project/strands_navigation/tree/hydro-devel/topological_navigation) where the task should be executed, plus the name of a [SimpleActionServer](http://wiki.ros.org/actionlib) to be called at the node and its associated arguments. Tasks must contain one of these, but not necessarily both.

To create a task, first create an instance of the `Task` message type. Examples are given in Python, as the helper functions currently only exist for Python, but C++ is also possible (and C++ helpers will be added if someone asks for them).

```python
from strands_executive_msgs.msg import Task
task = Task()
```

Then you can set the node id for where the task will be executed (or you can do this inline in the constructor):

```python
task.start_node_id = 'WayPoint1'
```
If you don't add a start node id then the task will be executed wherever the robot is located when it starts executing the task. If your task will end at a different location than it starts you can also specify `end_node_id`. This allows the scheduler to make better estimates of travel time between tasks.

To add the name of the action server, do:
```python
task.action = 'do_dishes'
```
Where 'do_dishes' is replaced by the action name argument you would give to the `actionlib.SimpleActionClient` constructor. If you do not specify an action, the executive will assume the task is to simply visit the location indicated by `start_node_id`.

You must also set the maximum length of time you expect your task to execute for. This is be used by the execution framework to determine whether your task is executing correctly, and by the scheduler to work out execution times. The duration is a `rospy.Duration` instance and is defined in seconds. 

```python
# wash dishes for an hour
dishes_duration = 60 * 60
task.max_duration = rospy.Duration(dishes_duration)
```

You can also specify the time window during which the task should be executed. 

```python
# don't start the task until 10 minutes in the future
task.start_after = rospy.get_rostime() + rospy.Duration(10 * 60)
# and give a window of three times the max execution time in which to execute
task.end_before = task.start_after + rospy.Duration(task.start_after.to_sec() * 3)
```


If the goal of the actionlib server related to your task  needs arguments, you must then add them to the task **in the order they are used in your goal type constructor**. Arguments are added to the task using the provided helper functions from [strands_executive_msgs.task_utils](https://github.com/strands-project/strands_executive/blob/hydro-release/strands_executive_msgs/src/strands_executive_msgs/task_utils.py). For example, for the following action which is available under [task_executor/action/TestExecution.action](https://github.com/strands-project/strands_executive/blob/hydro-devel/task_executor/action/TestExecution.action), you need to supply a string argument followed by a pose, then an int then a float.

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

 To add the string, do the following

```python
from strands_executive_msgs import task_utils
task_utils.add_string_argument(task, 'my string argument goes here')
```

For the pose, this must be added to the `mongodb_store message` store and then the `ObjectID` of the pose is used to communicate its location. This is done as follows

```python
from mongodb_store.message_store import MessageStoreProxy
msg_store = MessageStoreProxy()

p = Pose()
object_id = msg_store.insert(p)
task_utils.add_object_id_argument(task, object_id, Pose)
```

Ints and floats can be added as follows

```python
task_utils.add_int_argument(task, 24)
task_utils.add_float_argument(task, 63.678)
```


### Adding a Task

Tasks can be added to the task executor for future execution via the `add_tasks` service. These tasks are queued or scheduled for execution, and may not be executed immediately.

```python
add_tasks_srv_name = '/task_executor/add_tasks'
set_exe_stat_srv_name = '/task_executor/set_execution_status'
rospy.wait_for_service(add_tasks_srv_name)
rospy.wait_for_service(set_exe_stat_srv_name)
add_tasks_srv = rospy.ServiceProxy(add_tasks_srv_name, strands_executive_msgs.srv.AddTask)
set_execution_status = rospy.ServiceProxy(set_exe_stat_srv_name, strands_executive_msgs.srv.SetExecutionStatus)
    
try:
	# add task to the execution framework
    task_id = add_tasks_srv([task])
    # make sure the executive is running -- this only needs to be done once for the whole system not for every task
    set_execution_status(True)
except rospy.ServiceException, e: 
	print "Service call failed: %s"%e		
```

### Demanding a Task

If you want your task to be executed immediately, pre-empting the current task execution (or navigation to that task), you can use the `demand_task` service:

```python
demand_task_srv_name = '/task_executor/demand_task'
set_exe_stat_srv_name = '/task_executor/set_execution_status'
rospy.wait_for_service(demand_task_srv_name)
rospy.wait_for_service(set_exe_stat_srv_name)
demand_task_srv = rospy.ServiceProxy(demand_task_srv_name, strands_executive_msgs.srv.DemandTask)
set_execution_status = rospy.ServiceProxy(set_exe_stat_srv_name, strands_executive_msgs.srv.SetExecutionStatus)
    
try:
    # demand task execution immedidately
    task_id = demand_task_srv([task])
    # make sure the executive is running -- this only needs to be done once for the whole system not for every task
    set_execution_status(True)
except rospy.ServiceException, e: 
    print "Service call failed: %s"%e       
```

### Execution Information

The current execution status can be obtained using the service `strands_executive_msgs/GetExecutionStatus` typically on `/task_executor/get_execution_status`. True means the execution system is running, false means that the execution system has either not been started or it has been paused (see below).

To see the full schedule subscribe to the topic `/current_schedule` which gets the list of tasks in execution order. If `currently_executing` that means the first element of `execution_queue` is the currently active task. If it is false then the system is delaying until it starts executing that task.

To just get the currently active task, use the service `strands_executive_msgs/GetActiveTask` on `/task_executor/get_active_task`. If the returned task has a `task_id` of `0` then there is no active task (as you can't return `None` over a service).


### Interruptibility at Execution Time

By default the execution of tasks is interruptible (via actionlib preempt). Interruptions happen if another task is demanded while a task is running, or if the task exceeds its execution duration. If you do not wish your task to be interrupted in these condition you can provide the `IsTaskInterruptible.srv` service at the name `<task name>_is_interruptible`, e.g. `do_dishes_is_interruptible` from the example above. You can change the return value at runtime as this will be checked prior to interruption. 

Here's an example from the node which provides the `wait_action`.

```python

class WaitServer:
    def __init__(self):         
        self.server = actionlib.SimpleActionServer('wait_action', WaitAction, self.execute, False) 
        self.server.start()
        # this is not necessary in this node, but included for testing purposes
        rospy.Service('wait_action_is_interruptible', IsTaskInterruptible, self.is_interruptible)

    def is_interruptible(self, req):
        # rospy.loginfo('Yes, interrupt me, go ahead')
        # return True
        rospy.loginfo('No, I will never stop')
        return False

```

## Creating a Routine

Our use case for task execution is that the robot has a *daily routine* which is a list of tasks which it carries out every day. Such are routine can be created with the `task_routine.DailyRoutine` object which is configured with start and end times for the robot's daily activities:

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


The `DailyRoutine` declares the structure of the routine. The routine tasks must be passed to the `DailyRoutineRunner` to manage the creation of specific task instances and their addition to the task executor. 

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
