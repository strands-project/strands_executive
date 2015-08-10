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

You must also set the maximum length of time you expect your task to execute for. This is used by the execution framework to determine whether your task is executing correctly, and by the scheduler to work out execution times. The duration is a `rospy.Duration` instance and is defined in seconds. The max_duration field is not connected with any argument to the action server itself. 

```python
max_wait = 60 * 60
task.max_duration = rospy.Duration(max_wait)
```

Finally, you can set an argument 




- mapping between tasks and action servers
- get an action server running (wait_node)
- write a task spec and get it executed 
  -- execise (future execution)


