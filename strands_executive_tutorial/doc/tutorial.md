# The STRANDS Executive Framework -- Tutorial

This tutorial was originally written for the STRANDS Project [Long-term Autonomy for Mobile Robots Summer School 2015](https://lcas.lincoln.ac.uk/ecmr15/?q=lamor15).


## Preliminaries

In addition to assuming [a working knowledge of ROS](http://wiki.ros.org/ROS/Tutorials), this tutorial also assumes you are comfortable working with:

 * [actionlib](http://wiki.ros.org/actionlib) -- if not there are [tutorials here](http://wiki.ros.org/actionlib/Tutorials)
 * [mongodb_store](http://wiki.ros.org/mongodb_store) -- in particular the [message store](http://wiki.ros.org/mongodb_store#Message_Persistence:_message_store_node.py) which provides persistent storage of ROS messages.

 This tutorial also assumes you have the [strands_executive](https://github.com/strands-project/strands_executive) software installed. You should have done this when preparing for the summer school. If not, following the [installation instructions](https://github.com/strands-project/strands_executive#installation).

## Tasks 

This executive framework, schedules and manages the execution of *tasks*. An instance of a task object describes the desired exection of an [actionlib action server](http://wiki.ros.org/actionlib), usually within a *time window* and at a given robot location. In the first part of this tutorial you will create instances of tasks and manage their execution via the executive framework.

### The Wait Action 

To save the work of implementing an action server, we will work with one which is provided by the execution framework: the wait action. This action server simply either waits until a particular time or for a given duration before returning. The definition of action is as follows:

```
# the time to wait until
time wait_until
# if wait until is 0 then try waiting for this long, if this is 0 too, wait until explicitly woken or cancelled
duration wait_duration
---
---
# feedback message
float32 percent_complete
```

The server has three behaviours. If `wait_until` is specified, then the server will wait until this time until returning. If `wait_until` is not specified (i.e. it is a default `time` instance, one with 0 in both fields) then `wait_duration` is used to determine how long the server should wait before returning. If this is also unspecified (i.e. it is a default `duration` instance, one with 0 in both fields) then the server simply waits until a the `Empty` service `/wait_action/end_wait` is called and then returns. 

To see this in action, start the wait server as follows (assuming `roscore` is already running):

```bash
rosrun wait_action wait_node.py
```

You can then use the actionlib GUI client to explore different values:

```bash
rosrun actionlib axclient.py /wait_node
```

The `wait_duration` argument is self explanatory. The `wait_until` argument is seconds since epoch in UTC (which is one hour behind BST if you're doing this at the summer school). 

### Task Specification

Now we have an action server we'd like to execute, we can write a task to have its execution managed by the executive framework. To create a task, first create an instance of the `Task` message type. Examples are given in Python, as the helper functions currently only exist for Python, but C++ is also possible (and C++ helpers will be added if someone asks for them in the future).

```python
from strands_executive_msgs.msg import Task
task = Task()
```

To specify that this task should trigger the wait action, set the action field to the topic prefix of the action server (i.e. the string you use in the action server constructor, or the one you used as the last argument to axclient previously):

```python
task.action = '/wait_action'
```

You must also set the maximum length of time you expect your task to execute for. This is used by the execution framework to determine whether your task is executing correctly, and by the scheduler to work out execution times. The duration is a `rospy.Duration` instance and is defined in seconds. The `max_duration` field is not connected with any argument to the action server itself. 

```python
max_wait_minutes = 60 * 60
task.max_duration = rospy.Duration(max_wait_minutes)
```


As our action server requires arguments (i.e. wait_until or wait_duration), we must add these to the task too. Arguments must be added **in the order they are defined in your action message**. Arguments are added to the task using the helper functions from [strands_executive_msgs.task_utils](http://strands-project.github.io/strands_executive/strands_executive_msgs/html/namespacestrands__executive__msgs_1_1task__utils.html). For our wait_action, this means we must add a value for wait_until then a value for wait_duration (as this is the order defined in the action definition included above). The following code specifies an action that waits for 10 seconds. 

```python
task_utils.add_time_argument(task, rospy.Time())
task_utils.add_duration_argument(task, rospy.Duration(10))
```

Tasks can be assigned any argument type that is required by the action server. This is done via the mongodb_store and is explained in more detail [here](https://github.com/strands-project/strands_executive#tasks).

### Task Execution

We have now specified enough information to allow the task to be executed. For this to happen we must do three things. First, we must start up the execution system. The `strands_executive_tutorial` package contains a launch file which runs everything you need to get started: [mongodb_store](http://wiki.ros.org/mongodb_store); a [dummy topological navigation system](https://github.com/strands-project/strands_navigation/blob/indigo-devel/topological_utils/launch/dummy_topological_navigation.launch) (we will replace this with the robot's navigation system later); and the [executive framework](https://github.com/strands-project/strands_executive/blob/hydro-release/task_executor/launch/task-scheduler-mdp.launch) itself. Run this in a separate terminal as follows:

```bash
roslaunch strands_executive_tutorial tutorial_dependencies.launch
```

Second, we must tell the execution system that it can start executing task. This is done using the `SetExecutionStatus` service which can be used to pause and resume execution at runtime. The following provides functions which return the correct ROS service, after waiting for it to exist.

```python
def get_service(service_name, service_type):    
    rospy.loginfo('Waiting for %s service...' % service_name)
    rospy.wait_for_service(service_name)
    rospy.loginfo("Done")        
    return rospy.ServiceProxy(service_name, service_type)

def get_execution_status_service():
    return get_service('/task_executor/set_execution_status', SetExecutionStatus)
```

The resulting service can then be used to enable execution:

```python
set_execution_status = get_execution_status_service()
set_execution_status(True)
```

*Note*, that this only needs to be done once after the execution system has been started. If you don't set it to `True` then tasks can be added but nothing will happen. Once it has been set to `True` then it won't be set to `False` unless the service is called again (which will pause execution), or the task executor is shut down and started again.

Finally, we can request the *immediate* execution of the task using the `DemandTask` service. This service interrupts any task that is currently executing (provided the associated action server supports pre-emption) and starts the execution of the specified one.  The following code demonstrates this.

```python
def get_demand_task_service():
    return get_service('/task_executor/demand_task', DemandTask)

demand_task = get_demand_task_service()
demand_task(task) 
```

If you run all of the code provided so far (structured in an appropriate way), you should see some output like the following from the launch file you previously started:

```
[INFO] [WallTime: 1439285826.115682] State machine starting in initial state 'TASK_INITIALISATION' with userdata: 
	['task']
[INFO] [WallTime: 1439285826.118216] State machine transitioning 'TASK_INITIALISATION':'succeeded'-->'TASK_EXECUTION'
[INFO] [WallTime: 1439285826.118692] Concurrence starting with userdata: 
	['task']
[INFO] [WallTime: 1439285826.118842] Action /wait_action started for task 1
[WARN] [WallTime: 1439285826.121560] Still waiting for action server '/wait_action' to start... is it running?
[INFO] [WallTime: 1439285826.155626] Connected to action server '/wait_action'.
[INFO] [WallTime: 1439285826.192815] target wait time: 2015-08-11 10:37:16
[INFO] [WallTime: 1439285836.196404] waited until: 2015-08-11 10:37:16
[INFO] [WallTime: 1439285836.207261] Concurrent state 'MONITORED' returned outcome 'succeeded' on termination.
[INFO] [WallTime: 1439285837.166333] Concurrent state 'MONITORING' returned outcome 'preempted' on termination.
[INFO] [WallTime: 1439285837.181308] Concurrent Outcomes: {'MONITORED': 'succeeded', 'MONITORING': 'preempted'}
[INFO] [WallTime: 1439285837.181774] Action terminated with outcome succeeded
[INFO] [WallTime: 1439285837.186778] State machine transitioning 'TASK_EXECUTION':'succeeded'-->'TASK_SUCCEEDED'
[INFO] [WallTime: 1439285837.190386] Execution of task 1 succeeded
[INFO] [WallTime: 1439285837.190591] State machine terminating 'TASK_SUCCEEDED':'succeeded':'succeeded'
``` 

Most of this is output from the task execution node as it steps through the [finite state machine](https://github.com/strands-project/strands_executive/blob/hydro-release/task_executor/README.md#task-execution-and-monitoring) used for task execution. The following lines show the task is working as we want:

```
[INFO] [WallTime: 1439285826.192815] target wait time: 2015-08-11 10:37:16
[INFO] [WallTime: 1439285836.196404] waited until: 2015-08-11 10:37:16
```

### Task Scheduling

We can now extend our task specification to include information about *when* the task should be executed. This is an essential part of our long-term autonomy approach, which requires behaviours to happen at times specified either by a user, or the robot's on subsystems. Each task should be given a time window during which task execution should occur. The task scheduler which is part of the executive framework sequences all the tasks it is managing to ensure that the time window of every task is respected. If this is not possible then the most recently added tasks are dropped (unless you are using priorities, in which case lower priority tasks are dropped until a schedule can be found).

The opening time of the task's execution window is specified using the `start_after` field. The code below sets the start window to ten seconds into the future.

```python
task.start_after = rospy.get_rostime() + rospy.Duration(10)
```

The execution window should provide enough time to execute the task, but should also offer some slack as other tasks added to the system may have to executed first. The closing time of the the execution window is specified using the `end_before` field. The code below sets the end of the window such that the window is three times the maximum duration of the task. 

```python
task.end_before = task.start_after + rospy.Duration(task.max_duration.to_sec() * 3)
```

Previously we *demanded* task execution using the `DemandTask` service. This ignores the time window of the task and executes it immediately. To respect the time window we must use the service `/task_executor/add_tasks` of type `AddTasks`. 

```python
def get_add_tasks_service():
    return get_service('/task_executor/add_tasks', AddTasks)
```

This adds a list of tasks to the executive framework, which in turn triggers scheduling and updates the robot's execution schedule (assuming execution status has already been set to `True`). 

```python
add_tasks = get_add_tasks_service()
add_tasks([task])
```

When this script is executed, rather than the immediate execution of your task as previously, you should see the executor delay for some time (not quite ten seconds as there is an assumption of a minimum travel time before task execution) before executing the wait action. The output should be similar to that shown below, which shows the scheduler creating a schedule (which only contains the wait task) then the executor delaying execution until the star window of the task is open.

```
[ INFO] [1439548004.349319000]: SCHEDULER: Going to solve
[ INFO] [1439548004.397583000]: SCHEDULER: found a solution
[INFO] [WallTime: 1439548004.398210] start window: 11:26:53.412277
[INFO] [WallTime: 1439548004.398465]          now: 11:26:44.398102
[INFO] [WallTime: 1439548004.398639]  travel time:  0:00:02
[INFO] [WallTime: 1439548004.398825] need to delay 7.14174938 for execution
[INFO] [WallTime: 1439548004.399264] Added 1 tasks into the schedule to get total of 1
```

### Execution Information

You can use the following sources of information to inspect the current state of the executive framework.

The current execution status can be obtained using the service `GetExecutionStatus` on `/task_executor/get_execution_status`. A return value of `true` means the execution system is running, whereas `false` means that the execution system has either not been started or it has been paused.

To see the execution schedule, subscribe to the topic `/current_schedule` which gets the list of tasks in execution order. If `currently_executing` is `true` then this means the first element of `execution_queue` is the currently active task. If it is false then the system is delaying until it starts executing that task.

To just get the currently active task, use the service `strands_executive_msgs/GetActiveTask` on `/task_executor/get_active_task`. If the returned task has a `task_id` of `0` then there is no active task (as you can't return `None` over a service).

To get print outs in the terminal describing the operation of the executive framework, you can use the following scripts:

```bash
rosrun task_executor schedule_status.py
```

`schedule_status.py` prints out the current schedule and execution information. For example, for the above wait action, you might see something like

```
[INFO] [WallTime: 1439549489.280268] Waiting to execute /wait_action (task 3)
[INFO] [WallTime: 1439549489.280547] Execution to start at 2015-08-14 11:51:39
[INFO] [WallTime: 1439549489.280785] A further 0 tasks queued for execution
[INFO] [WallTime: 1439549494.295445] 

```

```bash
rosrun task_executor task_status.py
```

`task_status.py` prints out the events that occur internally in the framework. For one of our wait tasks, you might see the following:

```
task 3    /wait_action    ADDED   14/08/15 11:51:29
task 3    /wait_action    TASK_STARTED    14/08/15 11:51:37
task 3    /wait_action    EXECUTION_STARTED   14/08/15 11:51:37
task 3    /wait_action    EXECUTION_SUCCEEDED   14/08/15 11:51:47
task 3    /wait_action    TASK_SUCCEEDED    14/08/15 11:51:47
```

You can also subscribe to these events on the topic `/task_executor/events`.


### Exercise 1

a) Given what you know so far, create a script which schedules a configurable number of wait action tasks. All tasks should have the same duration and time window, but you should make sure that the time window is created such that it allows all the tasks to be executed. Use `schedule_status.py` and `task_status.py` to monitor the progress of execution and debug your script if necessary.

b) Optional. Create an action server which prints out all the details of a task, then uses this in place of the wait action for 1(a). See the [task documentation](https://github.com/strands-project/strands_executive#tasks) for how to add user-defined message types as task arguments.

### Tasks with locations

So far we have ignored the robot entirely. Now we will extend our task to feature the location of the robot when the task is performed. 


WayPoint3, WayPoint5, WayPoint6


- make sure they do the right order
- include travel time in the window -- try without and see what happens

- pause execution

