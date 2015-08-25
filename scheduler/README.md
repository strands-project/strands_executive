# Scheduler

A task scheduler implemented using mixed integer programming.

## Overview

This package provides single-robot task scheduling capabilities based on the algorithm presented in:

> L. Mudrova and N. Hawes. Task scheduling for mobile robots using interval algebra. In *2015 IEEE International Conference on Robotics and Automation (ICRA 2015)*, Seattle, Washington, USA, May 2015.

This extends prior work presented in 

> B. Coltin, M. Veloso, and R. Ventura. Dynamic user task scheduling for mobile robots. In *AAAI Workshop on Automated Action Planning for Autonomous Mobile Robots*. AAAI, 2011.


## Usage

If you want your tasks to be executed, then you should use the scheduler within the [scheduled_task_executor node](https://github.com/strands-project/strands_executive/blob/hydro-release/task_executor/README.md). If you wish to use it in a standalone fashion, you can call the `get_schedule` service described below, and demonstrated in the `scheduler/tests/test_scheduler_node.py`.

## Libraries

The library `libscheduler` provides the scheduling functionality outside of ROS. Use this if you want to include the scheduler in our own C++ library without using ROS. An example of its usage is provided in the `scheduler_example` executable, compiled from  `scheduler/src/main.cpp`.

## Nodes

### `scheduler_node`

The `scheduler_node` provides task scheduling as a ROS service. Examples of using this service are available in the tests for the scheduler `scheduler/tests/test_scheduler_node.py`.

#### Services

`get_schedule` ([strands_executive_msgs/GetSchedule](https://github.com/strands-project/strands_executive/blob/hydro-release/strands_executive_msgs/srv/GetSchedule.srv)) 

Given a list of tasks (`Task[] tasks`), this service call returns the order in which they should be executed and the desired execution start time of each task. Execution order is specified by the return value `uint64[] task_order` which contains the ids of input tasks in the order they should be executed. Execution times are provided by `time[] execution_times` which is in the same order as `task_order` and states the ideal time for task execution assuming all the input durations to the scheduler are as expected. The `DurationMatrix` argument specifies the travel time between task locations. It is a 2D array (implemented as a list of lists) where `durations[i].durations[j]` specifies the travel time between the end location of task `i` and the start location of task `j`.

#### Parameters

`~save_problems` (`bool`)

If true, the scheduling problems received by the scheduler (via `get_schedule`) are stored into the [mongodb_store message store](http://wiki.ros.org/mongodb_store#Message_Persistence:_message_store_node.py) collection 'scheduling_problems'. This allows for later replay via `rosrun scheduler replay_scheduling_problems.py`. Defaults to true.

`~scheduler_version` (`int`)

Switch the version of the algorithm the scheduler is running. If not specified, the Mudrova and Hawes algorithm is used. If the value 1 specified, then the original Coltin et al algorothm is used.

`~output_file` (`string`)

Path to a desired output file. If set, output from the scheduler is saved to this file.

`~timeout` (`int`)

How many seconds to allow the scheduler to run before timing out and returning (still potentially returning a result). The default value is to not have a timeout.
