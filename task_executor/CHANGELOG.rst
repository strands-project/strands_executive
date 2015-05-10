^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package task_executor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Added defaults for demanded task
* Simple test to check that navigation time is included in executor.
* Made execution schedule aware of navigation time
* Adding testing script of timings including navigation
* Contributors: Nick Hawes

0.0.24 (2015-05-05)
-------------------
* Made verbose the default
* Contributors: Nick Hawes

0.0.23 (2015-04-27)
-------------------
* This commit allows execution to recover from non-terminating or slow-to-terminate execution processes (either tasks or navigation).
* Filtering out unexecutable tasks from the routine.
  This has become necessary since the abilty to add daily tasks allows the addition of arbitrary tasks which are no longer bounded sensibly in time by the routine windows.
* Added end time to printout.
* Increased navigation timeout multiplier
  Also added a minimum timeout for all navigation and increased wiggle room on task execution duration.
* remove killer assert
* Moved print statement to after the None check.
  This prevents the error when printint on a None task.
* Fixed task event printer to use default timezone not utc.
* Contributors: Bruno Lacerda, Nick Hawes

0.0.22 (2015-04-21)
-------------------
* Added a verbose option to the schedule printer.
  If you do `rosparam set schedule_verbose true` you can now see the tasks which are scheduled. Use `rosparam set schedule_limit 10` etc. to limit the number of tasks printed.
* filtering extra daily tasks to remove impossible ones
* Utility functions for preceding commits.
* Added parameter `relaxed_nav` to prevent execution killing navigation if it tasks too long.
  `rosparam set relaxed_nav true` if you want your navigation actions to have a very long timeout. Set it back to false the timeouts will come from the predicted times.  This will only take effect on the next task.
* Added node that prints out task executive event.
  E.g.
  `rosrun task_executor task_status.py`
  shows
  ```
  task 2          WayPoint11      NAVIGATION_FAILED       19/04/15 18:55:04
  task 2          WayPoint11      TASK_FAILED     19/04/15 18:55:04
  task 3          WayPoint10      ADDED   19/04/15 18:55:17
  task 3          WayPoint10      TASK_STARTED    19/04/15 18:55:17
  task 3          WayPoint10      NAVIGATION_STARTED      19/04/15 18:55:17
  ```
* Script now prints out the routines and runtime.
* Added logging of routine start and stop. This is for better overall system analysis.
* Added ability to add tasks to the routine for just the day.
* Dealing with case where task added for scheduling has no start node.
  Tested in simulation and works here.
* mdp now uses ``topological_map_name `` parameter instead of getting it as an argument
* Dealing with case where task added for scheduling has no start node.
  Tested in simulation and works here.
* Contributors: Bruno Lacerda, Nick Hawes

0.0.21 (2015-04-15)
-------------------
* just change launch files for new name of wait_action, also changed default value to be interruptible
* Contributors: Lenka

0.0.20 (2015-04-12)
-------------------
* Merge branch 'hydro-release' of https://github.com/mudrole1/strands_executive into hydro-release
  Conflicts:
  task_executor/scripts/scheduled_task_executor.py
* Fixed some bugs in priorities handling, submitting testing file
* Added functionality of priorities and withdrawing tasks
* fixed bug in pairs causing scheduler to fail. Also fix bug with -1 constraint, which was causing that schedule was found for non existing solutions
  extended scheduled_task_executor to throw away tasks with  priorities
* try_schedule now tries to thow away some tasks in order to try to schedule smaller batch
* Removed fifo tester from make file.
  The fifo stuff is not actually used in the full system. Given that the scheduler test is in there now we are already testing all the things that this test.
* Fixed some bugs in priorities handling, submitting testing file
* Extended wait duration to see if that accounts for `#155 <https://github.com/strands-project/strands_executive/issues/155>`_
* Correcting order of values returned from demand task service call.
  Once the task_id number grew larger this was no longer interpreted (incorrectly) as a boolean, causing `#163 <https://github.com/strands-project/strands_executive/issues/163>`_.
  This fixes `#163 <https://github.com/strands-project/strands_executive/issues/163>`_.
* Removed deprecated code.
* Added locking in log methods to prevent concurrent calls to message store service. This should fix `#160 <https://github.com/strands-project/strands_executive/issues/160>`_
* removing frenap from dependencies
* removing frenap from launch file
* Added locking arond mdp expected time call so that code which calls it directly does not have concurrency issues with the other expected time call.
* Not using blank start_after for epoch.
  This should address `#157 <https://github.com/strands-project/strands_executive/issues/157>`_
* Added functionality of priorities and withdrawing tasks
* fixed bug in pairs causing scheduler to fail. Also fix bug with -1 constraint, which was causing that schedule was found for non existing solutions
  extended scheduled_task_executor to throw away tasks with  priorities
* Decreasing fudge factor now actual data is being used.
* Using full vector from mdp travel service.
  This closes `#152 <https://github.com/strands-project/strands_executive/issues/152>`_
* try_schedule now tries to thow away some tasks in order to try to schedule smaller batch
* Contributors: Bruno Lacerda, Lenka, Nick Hawes

0.0.19 (2015-03-31)
-------------------
* Added rostest for task_executor with scheduler
* Added param to task_executor to configure navigation type.
  Refactored launch and test files to use this flag.
* Switching to top nav in the fifo executor.
* Integrating MDP policy execution with switch to return to top nav if necessary.
* Integrated mdp travel time service.
  The current setup allows and code switch back to top nav if necessary. Tested with both.
  This also fixes a problem in the /mdp_plan_exec/get_expected_travel_times_to_waypoint service where it was expecting a duration for epoch but the service definition was of int.
* moved abstract_task_server into strands_executive_msgs and refactored wait_action
* made wait_action to use the new abstract_task_server as an example
* added an abstract_task_server
* Contributors: Marc Hanheide, Nick Hawes

0.0.18 (2015-03-23)
-------------------

0.0.16 (2014-11-26)
-------------------
* increasing timeout for nav
* Edited task allowed function to check task details.
* More none checking changes.
* Use `is None` instead of `not`.
  There's a reason it has been invented. This (and my next PR) probably fix the "local timezone doesn't work anymore" thing.
* Contributors: Bruno Lacerda, Lucas Beyer, Nick Hawes

0.0.15 (2014-11-23)
-------------------
* Added sanity checking to task routine.
* Handle case where action server for task does not exist
* Contributors: Nick Hawes

0.0.14 (2014-11-21)
-------------------
* Merge pull request `#113 <https://github.com/strands-project/strands_executive/issues/113>`_ from hawesie/hydro-release
  Changes to demanded tasks and failure cases.
* Changes to how on demand tasks are handled.
  The code that waited for a cancelled task had been commented out, leading to demanded tasks being ignored if something was currently executing. This addresses `#108 <https://github.com/strands-project/strands_executive/issues/108>`_.
* Added run dependency on wait_action.
  Fixes `#105 <https://github.com/strands-project/strands_executive/issues/105>`_.
  Conflicts:
  task_executor/package.xml
* Added locking to client end of expected time service call.
  This is for `#108 <https://github.com/strands-project/strands_executive/issues/108>`_.
* Contributors: Nick Hawes

0.0.13 (2014-11-21)
-------------------
* More robust handling of failure cases.
* Contributors: Nick Hawes

0.0.12 (2014-11-20)
-------------------
* Added bounds to repeat_every_delta method.
  Also cleaned up scheduled and executor output.
* Contributors: Nick Hawes

0.0.11 (2014-11-18)
-------------------
* Fixed bug with day start and end.
* Contributors: Nick Hawes

0.0.10 (2014-11-12)
-------------------

0.0.9 (2014-11-12)
------------------

0.0.8 (2014-11-12)
------------------
* Fixing up bugs in routine
* Added wait node back in.
* Updating task routine to be more flexible wrt window start and end times.
* Updated scheduled task executor with distance matrix parts and removed MDP depdendencies in sm base executor which I had previous forgotten.
* Contributors: Nick Hawes

0.0.7 (2014-11-07)
------------------
* Moving scripts to the install target rather than setup.py and the latter doesn't install them under the package name.
  Conflicts:
  task_executor/CMakeLists.txt
* Contributors: Nick Hawes

0.0.6 (2014-11-06)
------------------
* Updated and tested FIFO executor. Removed MDP depedency from base executor.
  This is now ready for a full release without the MDP parts.
* Contributors: Nick Hawes

0.0.5 (2014-11-01)
------------------
* Added launch file install target and disabled testing.
* Moving task_executor to release branch.
* Contributors: Nick Hawes

0.0.4 (2014-10-29 21:12)
------------------------

0.0.3 (2014-10-29 10:43)
------------------------

0.0.1 (2014-10-24)
------------------
* Removed task_executor from release branch
* Removed nodes that don't exist in this branch.
* This simply bulk replaces all ros_datacentre strings to mongodb_store strings inside files and also in file names.
* extended day to correct duration
* Added repeat every mins repeat.
* Added first task logic to scheduler.
  Also made replay script work with mulitple parallel schedulers.
* Adding feedback to test action node.
* Adding timeout to scheduler.
* Some different printing
* Added autonomy percentage calculation.
* Added day counting.
* Fixed bug with wrong duration check.
* Fixed problem with duplicate ends to events.
* Adding some more counts to query.
* Added query for execution time.
* Task events are now published to `/task_executor/events` as they happen.
  This can be used to for a task GUI later. To get a console overview, see `rosrun task_executor task_event_printer.py`
* Restructured query code.
* Added argparse and result on empty
* Added summary printing script
* example to add extinguisher check task
* starts scheduling 15 min before task should be executed, instead of 1 hour
* REALLY getting correct outcomes from concurrency container
* getting correct outcomes from concurrency containers
* script to add task
* Merge branch 'sm_executor' of https://github.com/hawesie/strands_executive into sm_executor
  Conflicts:
  task_executor/src/task_executor/base_executor.py
* Fixed minor scheduling issues.
  1) Made service calls thread safe.
  2) Fixed order of calls in cancellation
  3) Removed blocking assumption in demand task in scheduler
  4) Changed bounding of tasks based on current execution time.
* Logging working from state machine now.
* Working preempts on action too.
  Seems clean and robust for now.
* Nav prempt working with concurrence.
* Added cancellation timeout.
  This also checks if we get late preempt responses.
* First pass of executor based on smach working.
* Building FSM executor
* Added a stricter cancel for navigation and execution.
  This new version does not wait to receive a callback from the cancelled action server. This is dangerous in that the next task may start while the previous task is still ending, but there isn't a huge problem with this in our current tasks. A better solution would be to wait a bit, then give up on waiting for the callback, but this is hard in the current design. Probably needs to be reimplemented as a state machine to make this cleaner.
* Updated test executor to match mdp expectations, so now uses monitored navigation.
* Changes for local testing.
* Output changes
* Merge branch 'sm_executor' of https://github.com/hawesie/strands_executive into sm_executor
  Conflicts:
  mdp_plan_exec/scripts/mdp_planner.py
  task_executor/src/task_executor/base_executor.py
* Fixed minor scheduling issues.
  1) Made service calls thread safe.
  2) Fixed order of calls in cancellation
  3) Removed blocking assumption in demand task in scheduler
  4) Changed bounding of tasks based on current execution time.
* Logging working from state machine now.
* Working preempts on action too.
  Seems clean and robust for now.
* Nav prempt working with concurrence.
* Added cancellation timeout.
  This also checks if we get late preempt responses.
* First pass of executor based on smach working.
* Building FSM executor
* Added a stricter cancel for navigation and execution.
  This new version does not wait to receive a callback from the cancelled action server. This is dangerous in that the next task may start while the previous task is still ending, but there isn't a huge problem with this in our current tasks. A better solution would be to wait a bit, then give up on waiting for the callback, but this is hard in the current design. Probably needs to be reimplemented as a state machine to make this cleaner.
* Updated test executor to match mdp expectations, so now uses monitored navigation.
* Changes for local testing.
* Output changes
* - default timeout for navigation setr for 10 min.
  - catching datacentre insert exceptions
* added example for fire extinguisher task in example demand tasks
* REmoved asserts for long-term stability.
* Catching killer exception.
* Added scaling for nav timeout.
* Reconnecting on nav start and returning empty responses correctly.
* making sure some initial node is given for the expected time
* Added days and dates off for the routine.
  Not the most efficient way to bring in the check, but this part is tested for this kind of behaviour.
* Adding support for giving the robot days off.
* Working around the time comparison bug some more.
* Added bool type to task
* Reduced calls to mdp time stuff.
* working with new mdp exec
* Merge branch 'hydro-devel' of https://github.com/hawesie/strands_executive into logging
  Conflicts:
  task_executor/launch/task-scheduler.launch
  task_executor/scripts/task_routine_tester.py
  task_executor/src/task_executor/base_executor.py
  task_executor/src/task_executor/utils.py
* Updated after merge.
* Integrated @BFALacerda's latest changes.
* moved some bits around
* Makes pretend navigation more realistic for node changes.
* Minor logging changes.
* Fixes for very short navigation times.
* Added logging of task event changes to message store.
* Added nav timeout, but not fully tested.
  Seems to be a problem when there is a clear_schedule call during nav which doesn't respond to prempt immediately. It actually seems to be due to the (faked) nav returning normal when it should be preempted.
* Added mdp expected time to base_executor
* Added logging of task event changes to message store.
* Update continuous_patrolling.py
  getting map from topological_maps collection
* Added nav timeout, but not fully tested.
  Seems to be a problem when there is a clear_schedule call during nav which doesn't respond to prempt immediately. It actually seems to be due to the (faked) nav returning normal when it should be preempted.
* Added mdp expected time to base_executor
* minimal changes for the executor to start using the policy generation/execution for navigating
* Added callback for checking whether tasks should be sent to scheduler.
  This is used to prevent new tasks being sent when battery is low.
  Also made task_routine killable further.
* Fixed bug with incomplete comparison.
* Fixed some bugs in demanding tasks and added cancellation services.
* Added start and end day callbacks.
  Also make task routine ctrl-c-able.
* Merge branch 'hydro-devel' of https://github.com/strands-project/strands_executive into hydro-devel
* Merge branch 'hydro-devel' of https://github.com/strands-project/strands_executive into hydro-devel
* Removed potential infinite loop.
* Clarified behaviour around rescheduling after a demand.
  Dropping of out-of-bounds additional tasks are not handled separately to out-of-bounds previously scheduled tasks.
* prism updated, big fixes, adding mdp_planner to launch file
* saving prism files to temp dir
* getting example task routines to have proper start and ending points
* Merge branch 'hydro-devel' of https://github.com/strands-project/strands_executive into hydro-devel
  Conflicts:
  scheduler/src/scheduler.cpp
  task_executor/scripts/example_task_routine.py
  Conflicts solved by mostly using what was upstream
* allowing to change initial state for expected travel times
* Fixed demand tasks when delaying for next execution.
  Also added smalls script to summarise the current schedule.
* On demand tasks working.
  Also added in time and duration types for tasks.
  After a demand the scheduler tries to schedule back in the previously scheduled but unexecuted tasks. If this is not successful then these tasks are dropped. If these are successfully scheduled back in then it also tries to schedule back in the task which was interrupted by the demand. If this is not possible only the interrupted task is dropped.
  Demands can be interrupted by timeout and by subsequent demanded tasks.
* Demanded tasks are executed. Others are cleared and cancelled.
* Changes for on demand tasks.
  Added service for on-demand tasks.
  Restructued scheduled executor to separate new and old tasks, with the aim to allow this to be used to recover tasks overridden by on-demand requests.
* Adding prism and initial prism-ros interaction
* Added correct import
* Added timeout cancellation to base executor.
  * This uses rospy.Timer which has looked odd under simulation time.
  * Also refactored test action server into separate file.
  This closes `#17 <https://github.com/strands-project/strands_executive/issues/17>`_.
* Success and failure now noted.
* Delayed execution tasks now working correctly with timer.
* Publishing schedule and handling scheduler fail.
* Updated patroller script. Added instructions to readme.
* Set up for just patrolling. Launch file printing to screen sensible amounts.
* Added launmch file.
* Added launch file.
* Added stuff on DailyRoutine to the README.
* Example routine produces more-or-less the expected behaviour.
* Trying to get routine adding tested.
* Moved to adding tasks in a batch. Old interface left for compatibility.
* Routine object now can be used to generate conveniet routines.
* Delaying the tasks for the scheduler appears to work.
* Updated to use timezones, and managing intervals.
* Adding prototypes for scheduling tasks.
* Looking to add time delays to scheduler and executor, but bug found in scheduler.
* Fixed node_id attribute name change
* Actions executed from schedule, but only first.
  Fixed bug in base_executor where missing wait_for_server() caused silent blocking.
* Scheduled execution almost up to actually executing things.
* Example task client now registers a bunch of timed tasks for scheduling.
* Setting up scheduler tests.
* Tester in place
* Running scheduler, receiving back at execution framework.
* Working calls to the scheduler!
* Scheduler C++ node is now called with tasks.
* Expanding schedule executor.
* Adding infrastructure for scheduled execution.
* Updated tests for new action definition.
* using new TopologicalNode.msg
* Added int and float arguments to task execution.
* Using proper nodes from datacentre via ros param.
* Added launch file for patrolling.
* Added basic script to propose patrol targets.
* Added the ability to just drive somewhere without doing an action.
* Test now include navigation, and is working.
* Navigation added for faked action server.
* Removed creation of service based on node name (silly!).
  This now fixes rostest integration. Run with `rostest task_executor fifo_tester.test`
* Basic test of FIFO done and working.
  Works from the command line, but can't seem to make the rostest integration work.
* Basic FIFO executor working without preemption/pausing or navigation to points.
* Basic execution flow through abstract and FIFO working.
* Abstracted basic functionality into base class
* Working call with action arguments.
* Moved test action to task_executor, adding server to provide it.
* Basic node comms working.
* Working basic task creation.
* Added messages and structure.
* Contributors: Bruno Lacerda, Chris Burbridge, Nick Hawes
