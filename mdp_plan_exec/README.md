# MDP Plan Exec

An optimal, high-level robot route planner and travel time estimator, using probabilistic model-checking techniques.

## Overview

This package provides single-robot optimal route planning capabilities, along with execution time expectations based on the method presented on:

> B. Lacerda, D. Parker, and N. Hawes. Optimal and Dynamic Planning for Markov Decision Processes with Co-Safe LTL Specifications In *20114 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2014)*, Chicago, Illinois, USA, September 2014.

This uses MDP models of the environment and an [extension](https://github.com/bfalacerda/prism-robots) of the probabilistic model checker tool [PRISM](http://www.prismmodelchecker.org/) that allows for communication with a ROS node via sockets, to generate the cost-optimal policies.


In order to fill the costs and probabilities of the MDP models with predictions from real-world data, we use the approach presented on:

> J. Fentanes, B. Lacerda, T. Krajn&iacute;k, N. Hawes, and M. Hanheide. Now or later? Predicting and Maximising Success of Navigation Actions from Long-Term Experience. In *2015 IEEE International Conference on Robotics and Automation (ICRA 2015)*, Seattle, Washington, USA, May 2015.


## Usage

You can use this package ithin the [scheduled_task_executor node](https://github.com/strands-project/strands_executive/blob/hydro-release/task_executor/README.md). This allows for a  complete framework that allows scheduling and execution of tasks taking into account the time of day. If you wish to use it in a standalone fashion, you can  do ``roslaunch mdp_plan_exec mdp_plan_exec.launch topological_map:=t`` where ``t``is the name of the map saved in the [``mongo_db``](https://github.com/strands-project/mongodb_store) using the [``topological_utils``](https://github.com/strands-project/strands_navigation/tree/indigo-devel/topological_utils) package. 

## Nodes

### `special_waypoints_manager.py`

#### Services

`add_delete_special_waypoint` ([strands_executive_msgs/AddDeleteSpecialWaypoint](https://github.com/strands-project/strands_executive/blob/hydro-release/strands_executive_msgs/srv/AddDeleteSpecialWaypoint.srv)) 

Adds "special" waypoints. These can be "forbidden" waypoints (i.e., waypoints to avoind during task execution, e.g., for safety reasons), or "safe" waypoints, for which the robot should navigate to when told to get out of the way by an end-user. These sets of waypoints are used to automatically generate the co-safe LTL specifications for the action server provided by the  `mdp_travel_time_estimator.py` node (see below). The boolean `is_addition` defines if the service call is adding or removing a waypoint, and the `waypoint_type`constant defines if the waypoint in the service call is forbidden or safe.


`get_special_waypoints` ([strands_executive_msgs/GetSpecialWaypoints](https://github.com/strands-project/strands_executive/blob/hydro-release/strands_executive_msgs/srv/GetSpecialWaypoints.srv)) 

Returns the current lists of forbidden and safe waypoints, along with the corresponding LTL subformulas. For safe waypoints these are of the form 

$$
\Gamma(z) = \int_0^\infty t^{z-1}e^{-t}dt\,.
$$

MUDAR Given a list of tasks (`Task[] tasks`), this service call returns the order in which they should be executed and the desired execution start time of each task. Execution order is specified by the return value `uint64[] task_order` which contains the ids of input tasks in the order they should be executed. Execution times are provided by `time[] execution_times` which is in the same order as `task_order` and states the ideal time for task execution assuming all the input durations to the scheduler are as expected. The `DurationMatrix` argument specifies the travel time between task locations. It is a 2D array (implemented as a list of lists) where `durations[i].durations[j]` specifies the travel time between the end location of task `i` and the start location of task `j`.

### `mdp_travel_time_estimator.py`

#### Services

`get_expected_travel_times_to_waypoint` ([strands_executive_msgs/GetExpectedTravelTimesToWaypoint](https://github.com/strands-project/strands_executive/blob/hydro-release/strands_executive_msgs/srv/GetExpectedTravelTimesToWaypoint.srv)) 

MUDAR Given a list of tasks (`Task[] tasks`), this service call returns the order in which they should be executed and the desired execution start time of each task. Execution order is specified by the return value `uint64[] task_order` which contains the ids of input tasks in the order they should be executed. Execution times are provided by `time[] execution_times` which is in the same order as `task_order` and states the ideal time for task execution assuming all the input durations to the scheduler are as expected. The `DurationMatrix` argument specifies the travel time between task locations. It is a 2D array (implemented as a list of lists) where `durations[i].durations[j]` specifies the travel time between the end location of task `i` and the start location of task `j`.

### `mdp_policy_executor.py`

Executes policies
The `scheduler_node` provides task scheduling as a ROS service. Examples of using this service are available in the tests for the scheduler `scheduler/tests/test_scheduler_node.py`.

#### Actions

`execute_policy` ([strands_executive_msgs/ExecutePolicy](https://github.com/strands-project/strands_executive/blob/hydro-release/strands_executive_msgs/action/ExecutePolicy.action)) 

bla

