# The STRANDS Executive Framework -- Tutorial

This tutorial was originally written for the STRANDS Project [Long-term Autonomy for Mobile Robots Summer School 2015](https://lcas.lincoln.ac.uk/ecmr15/?q=lamor15).


## Preliminaries

In addition to assuming [a working knowledge of ROS](http://wiki.ros.org/ROS/Tutorials), this tutorial also assumes you are comfortable working with:

 * [actionlib](http://wiki.ros.org/actionlib) -- if not there are [tutorials here](http://wiki.ros.org/actionlib/Tutorials)
 * [mongodb_store](http://wiki.ros.org/mongodb_store) -- in particular the [message store](http://wiki.ros.org/mongodb_store#Message_Persistence:_message_store_node.py) which provides persistent storage of ROS messages.

 This tutorial also assumes you have the [strands_executive](https://github.com/strands-project/strands_executive) software installed. You should have done this when preparing for the summer school. If not, following the [installation instructions](https://github.com/strands-project/strands_executive#installation).

## Tasks 

This executive framework, schedules and manages the execution of *tasks*. An instance of a task object describes the desired exection of an [actionlib action server](http://wiki.ros.org/actionlib), usually within a *time window* and at a given robot location. In the first part of this tutorial you will create instances of tasks and manage their execution via the executive framework.

**Note**, whilst any actionlib server can be used with the execution framework, it is important that the server correctly responds to preemption requests.

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
#!/usr/bin/env python

import rospy

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
from strands_executive_msgs import task_utils

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
from strands_executive_msgs.srv import AddTasks, DemandTask, SetExecutionStatus

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

### Tasks with Locations

So far we have ignored the robot entirely. Now we will extend our task to feature the location of the robot when the task is performed. Each task can optionally have a start location and an end location. Locations are names of nodes in the robot's topological map (which you should have worked with yesterday). If the start location is provided, the task executive drives the robot to this location before starting the execution of the task action. If navigation fails, the task will not be executed. The end location describes the location the executive system expects the robot to be at when the task execution is completed. If left blank, this is assumed to be the same as the start location. The start and end locations are used by the scheduler to determine what order to execute the tasks.

*For this part of the tutorial, we will work with the UOL MHT simulation and the associated topological map. From here on the examples assume you are running the correct mongodb store, simulation and navigation launch files for this simulated setup.*

Since you now have a simulated robot running (see note above), you now **should not** use the `tutorial_dependencies` launch file. Instead, launch the executive framework as follows:

```bash
roslaunch task_executor task-scheduler.launch scheduler_version:=1

```

The `scheduler_version:=1` parameter ensures the optimal version of the scheduler is used (the original Coltin et al. algorithm). This version produces optimal schedules (including the travel time between locations), although it starts to take a long time to produce schedules once you go above 20 or so tasks in a single scheduling problem. You can omit this parameter to run the non-optimal version which uses preprocessing to produce non-optimal schedules quickly.

To configure a task's location, set the following fields in the task object:

```python
task.start_node_id = `Waypoint3`
task.end_node_id = `Waypoint3`
```

Make sure you add the above lines to your existing code *before* the task is send to the scheduler. You can then use the `add_tasks` service as before, and you should see the robot drive to `Waypoint3` before executing a wait action (assuming you're extending the code from above).

**Note** that now the robot must travel between locations you need to be make sure your time windows for execution are updated appropriately. The time window specifies the time in which the action is executed, not the navigation time. However, whereas you could previously easily fit two 10 second wait tasks in a 30 second time window (if you gave them both the same window), if it takes 120 seconds to travel between the two locations then the scheduler will not be able to include them both in the same schedule as the travel time between them will place one task outside its time window. 

### Exercise 2

a) Extend your previous multi-task code to include locations in the tasks, with the robot travelling to at least three different locations.

### Changes to Execution

You now know how to have a list of tasks scheduled and executed. If a task is *demanded* using the `DemandTask` service (as in the first part of this tutorial), the currently executing task is interrupted and the demanded one is executed in its place. In parallel, a new schedule is created which contains the remaining scheduled tasks and the interrupted one if possible (i.e. if the time windows can be respected). In all cases, if all tasks cannot be scheduled some are dropped until a valid schedule can be produced. 

You can also pause the execution of the current task by sending a value of `false` to the `SetExecutionStatus` service. This will pause the task execution in the current state it is in, either navigation or action execution, until a value of `true` is sent to the service.

If you have written a task which should not be interrupted by these methods, you can create a service which informs the executive whether or not interruption is possible. The instructions for this are [here](https://github.com/strands-project/strands_executive#interruptibility-at-execution-time).

### Exercise 3

a) Use your previous code to start the simulated robot executing scheduled tasks. Once it is executing, experiment with setting the execution status and demand tasks. Monitor the effects of these interactions using 

```bash
rosrun task_executor schedule_status.py
```

You can alter the execution status from the command line using 

```bash
rosservice call /task_executor/set_execution_status "status: true"
```

### Routines

For long-term operation, we often want the robot to execute tasks according to some *routine*, e.g. visit all the rooms in the building between 9am and 11am. The STRANDS executive framework supports the creations of such routines through two mechanisms. The `DailyRoutine` and `DailyRoutineRunner` classes (described [here](https://github.com/strands-project/strands_executive#creating-a-routine)) support the creation of routines of daily repeating tasks. The `RobotRoutine` class (described [here](https://github.com/strands-project/strands_executive_behaviours/tree/hydro-devel/routine_behaviours#routine-behaviours)) builds on these objects to provide managed routine behaviour for a robot. This class also manages the battery level of the robot, provides a hook for when the robot is idle (i.e. it has no tasks to execute in the near future), and also manages the difference between day and night for the robot (at night the robot docks on a charger and can then only execute tasks without movement). 

The `PatrolRoutine` class provides an example subclass of `RobotRoutine` which simply generates a patrol behaviour which creates tasks to wait at every node in the topological map. You can see this in operation by running:

```bash
rosrun routine_behaviours patroller_routine_node.py
```

With this node running you should see that the robot creates a schedule to visit all the nodes, or it fails to create on (if the time windows are not satisfiable) and instead visits a random node when idle.

Before going further, we can use the code from this node to understand the basic properties of a routine.

```python
import rospy

from datetime import time
from dateutil.tz import tzlocal

from routine_behaviours.patrol_routine import PatrolRoutine
    

if __name__ == '__main__':
    rospy.init_node("patroller_routine")

    # start and end times -- all times should be in local timezone
    localtz = tzlocal()
    start = time(8,00, tzinfo=localtz)
    end = time(20,00, tzinfo=localtz)

    idle_duration=rospy.Duration(20)

    routine = PatrolRoutine(daily_start=start, daily_end=end, idle_duration=idle_duration)    
     
    routine.create_routine()
 
    routine.start_routine()

    rospy.spin()

```

Looking at the code in more detail:

```python
    start = time(8,00, tzinfo=localtz)
    end = time(20,00, tzinfo=localtz)
```

The above code defines the working day of the robot. No task execution will happen before `start`, and when `end` is reached, all execution will cease and the robot will dock for the night.

```python
    idle_duration=rospy.Duration(20)
```

The above code is used to configure how long the robot should not be executing a task before it considers itself idle.

```python
    routine = PatrolRoutine(daily_start=start, daily_end=end, idle_duration=idle_duration)    
```

This creates the routine object, passing the defined values.

```python

   routine.create_routine()
 
   routine.start_routine()
```

`create_routine()` runs code which specifies the tasks and populates the routine within the object. You can see the code for this [here](https://github.com/strands-project/strands_executive_behaviours/blob/hydro-devel/routine_behaviours/src/routine_behaviours/patrol_routine.py#L84). Following this, `start_routine` triggers the execution of the populated routine from the previous step.

If you want to create a routine that performs tasks at all, or a selection of, nodes in the map, you can create a subclass of `PatrolRoutine` and override methods such as `create_patrol_routine` and `create_patrol_task` to create task-specific behaviour. For the final part of this tutorial we will eschew this approach, and instead create a simple routine from `RobotRoutine`.  Use the following code as a structure for your routine.

```python

#!/usr/bin/env python

import rospy

from routine_behaviours.robot_routine import RobotRoutine

from datetime import time, date, timedelta
from dateutil.tz import tzlocal

from strands_executive_msgs.msg import Task
from strands_executive_msgs import task_utils


class ExampleRoutine(RobotRoutine):
    """ Creates a routine which simply visits nodes. """

    def __init__(self, daily_start, daily_end, idle_duration=rospy.Duration(5), charging_point = 'ChargingPoint'):
        # super(PatrolRoutine, self).__init__(daily_start, daily_end)        
        RobotRoutine.__init__(self, daily_start, daily_end, idle_duration=idle_duration, charging_point=charging_point)

    def create_routine(self):
        pass        

    def on_idle(self):
        """
            Called when the routine is idle. Default is to trigger travel to the charging. As idleness is determined by the current schedule, if this call doesn't utlimately cause a task schedule to be generated this will be called repeatedly.
        """
        rospy.loginfo('I am idle')    



if __name__ == '__main__':

    rospy.init_node('tutorial_4')

    # start and end times -- all times should be in local timezone
    localtz = tzlocal()
    start = time(8,00, tzinfo=localtz)
    end = time(20,00, tzinfo=localtz)

    # how long to stand idle before doing something
    idle_duration=rospy.Duration(20)

    routine = ExampleRoutine(daily_start=start, daily_end=end, idle_duration=idle_duration)    
     
    routine.create_routine()
 
    routine.start_routine()

    rospy.spin()
```

This is similar to the `patroller_routine_node` code we saw previously, except we now see the (currently empty) implementation of some parts of routine class. If you run this (alongside the executive framework, simulation, navigation etc.) you should see something like the follows:


```
[INFO] [WallTime: 1440522933.015355] Waiting for task_executor service...
[INFO] [WallTime: 1440522933.018245] Done
[INFO] [WallTime: 1440522933.023142] Fetching parameters from dynamic_reconfigure
[INFO] [WallTime: 1440522933.027528] Config set to 30, 10
[INFO] [WallTime: 1440522933.533364] Current day starts at 2015-08-25 08:00:00+01:00
[INFO] [WallTime: 1440522933.533773] Current day ends at 2015-08-25 20:00:00+01:00
[INFO] [WallTime: 1440522933.547498] extra_tasks_for_today
[INFO] [WallTime: 1440522933.547924] Scheduling 0 tasks now and 0 later
[INFO] [WallTime: 1440522933.548184] triggering day start cb at 2015-08-25 18:15:33.547396+01:00
[INFO] [WallTime: 1440522933.548409] Good morning
[INFO] [WallTime: 1440522933.554520] Scheduling 0 tasks now and 0 later
[INFO] [WallTime: 1440522958.590649] I am idle
```

The `I am idle` line should appear after 20 seconds, as configured by `idle_duration`. Nothing else should happen, as we have not yet added tasks. 

To add tasks, we will fill in the `create_routine` method. In this method we will use the instance of `DailyRoutine` contained within `RobotRoutine`. Tasks are added using the `repeat_every*` methods from this object. These take a list of tasks and store them such that they can be correctly instantiated with start and end times every day. When creating tasks for a routine, you should not specify `start_after` or `end_before` as these will be determined by the routine itself. Let's assume we have a function called `wait_task_at_waypoint` which creates a `Task` object to perform a  10 second wait at a given waypoint. We can then get the routine to schedule two such wait tasks to be performed every day.


```python
    def create_routine(self):
        daily_wps = ['WayPoint6', 'WayPoint3']
        daily_wp_tasks = [wait_task_at_waypoint(wp) for wp in daily_wps]
        self.routine.repeat_every_day(daily_wp_tasks)
```

If you use this code in your previous script, you should see the robot execute the two tasks, then go idle. If you wait long enough (until the start of the next day!) the robot will repeat this execution. Rather that wait that long, let's add some tasks with a shorter repeat rate into `create_routine`.

```python
        minute_wps = ['WayPoint5']
        minute_wp_tasks = [wait_task_at_waypoint(wp) for wp in minute_wps]
        self.routine.repeat_every_delta(minute_wp_tasks, delta=timedelta(minutes=5))
```

This causes a task to wait at the given waypoint to be scheduled at five minute intervals throughout the day. These tasks will be scheduled alongside the ones from the `daily_wp_tasks` (provided you didn't remove the code). `repeat_every_delta` is the most flexible of the routine creation tools, but you can also use [other methods from `DailyRoutine`](http://strands-project.github.io/strands_executive/task_executor/html/classtask__executor_1_1task__routine_1_1DailyRoutine.html).

### Exercise 4

a) Create a routine that mixes tasks with hourly and daily repeats.

b) Add tasks that either happen in the morning or afternoon. You will need to use the `start_time` argument to `repeat_every`. 

c) Once you have all of this working in simulation, move your code to your group's robot. Create a routine to patrol the nodes in your topological map. Extend this code to create meta-room maps at certain waypoints at regular intervals. You can also try creating your own tasks/action servers, or scheduling the execution of existing ones (speech, PTU movement, topological edge traversal etc.).

## Additional Resources

 * [framework README](https://github.com/strands-project/strands_executive/blob/hydro-release/README.md)

 * [task_executor documentation](https://github.com/strands-project/strands_executive/blob/hydro-release/task_executor/README.md) and [API](http://strands-project.github.io/strands_executive/task_executor/html/namespaces.html)

 * [scheduler documentation](https://github.com/strands-project/strands_executive/blob/hydro-release/scheduler/README.md) 

 * [routine_behaviours documentation](https://github.com/strands-project/strands_executive_behaviours/blob/hydro-devel/routine_behaviours/README.md) and [API](http://strands-project.github.io/strands_executive_behaviours/routine_behaviours/html)

