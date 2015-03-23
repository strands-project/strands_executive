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

Dynamically manages special waypoints defined, for example, by an end user.

#### Services

`/mdp_plan_exec/add_delete_special_waypoint` ([strands_executive_msgs/AddDeleteSpecialWaypoint](https://github.com/strands-project/strands_executive/blob/hydro-release/strands_executive_msgs/srv/AddDeleteSpecialWaypoint.srv)) 

Adds "special" waypoints. These can be "forbidden" waypoints (i.e., waypoints to avoind during task execution, e.g., for safety reasons), or "safe" waypoints, for which the robot should navigate to when told to get out of the way by an end-user. These sets of waypoints are used to automatically generate the co-safe LTL specifications for the action server provided by the  `mdp_travel_time_estimator.py` node (see below). The boolean `is_addition` defines if the service call is adding or removing a waypoint, and the `waypoint_type`constant defines if the waypoint in the service call is forbidden or safe.


`/mdp_plan_exec/get_special_waypoints` ([strands_executive_msgs/GetSpecialWaypoints](https://github.com/strands-project/strands_executive/blob/hydro-release/strands_executive_msgs/srv/GetSpecialWaypoints.srv)) 

Returns the current lists of forbidden and safe waypoints, along with the corresponding LTL subformulas. For safe waypoints the corresponding subformula is the disjunction of all waypoints in the safe waypoint list, and for forbidden waypoints the corresponding subformula is the conjunction of negation of all waypoints in the forbidden waypoints list.


### `mdp_travel_time_estimator.py`

Provides expected times to a target waypoint, by performing value iteration on a product MDP obtained from the MDP model representing the topological map.

#### Services

`/mdp_plan_exec/get_expected_travel_times_to_waypoint` ([strands_executive_msgs/GetExpectedTravelTimesToWaypoint](https://github.com/strands-project/strands_executive/blob/hydro-release/strands_executive_msgs/srv/GetExpectedTravelTimesToWaypoint.srv)) 

Given a string `target_waypoint` identifying an element of  the topological map, and a ROS timestamp, this service returns the expected times to travel from all waypoints to the target waypoint. These are enconded by vectors `string[] source_waypoints` and `float64[] travel_times`, where `travel_times[i]`represents the expected travel time from `source_waypoints[i]`to `target_waypoint`.

### `mdp_policy_executor.py`

Generates and executes a policy for a given input task,  by performing value iteration on a product MDP obtained from the MDP model representing the topological map.

#### Actions

`/mdp_plan_exec/execute_policy` ([strands_executive_msgs/ExecutePolicy](https://github.com/strands-project/strands_executive/blob/hydro-release/strands_executive_msgs/action/ExecutePolicy.action)) 

Given a `task_type` and `target_id`, generates a cost optimal policy and executes the navigation actions associated to it. The specification is a co-safe LTL formula generated taking into account the `task_type`. In the following `forbidden_waypoints_ltl_string` and `safe_waypoints_ltl_string` are the strings obtained from the `get_special_waypoints`service:

* `task_type = GOTO_WAYPOINT`. This generates and executes a policy for formula `F target_id` (i.e., "reach `target_id`") if there are no forbidden  waypoints, or for formula `forbidden_waypoints_ltl_string U target_id`  (i.e., reach `target_id`while avoiding the forbidden waypoints).
* `task_type = LEAVE_FORBIDDEN_AREA`. This generates and executes a policy for formula `F forbidden_waypoints_ltl_string` (i.e., get out of the forbidden waypoints as soon as possible). It should be called when the robot ends up in forbidden areas of the environment due to some failure in execution.
* `task_type = GOTO_CLOSEST_SAFE_WAYPOINT`. This generates and executes a policy for formula `F safe_waypoints_string` (i.e., reach a safe waypoint as soon as possible). It should be called when the robot is sent to a safe zone, for example by an end-user.
* `task_type = COSAFE_LTL`. This generates and executes a policy for formula `target_id`. In this case, `target_id`is a general co-safe LTL formula written in the [PRISM specification language](http://www.prismmodelchecker.org/manual/PropertySpecification/SyntaxAndSemantics).

The action server also provides the expected execution time as feedback.

