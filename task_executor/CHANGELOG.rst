^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package task_executor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.1 (2017-09-15)
------------------

1.2.5 (2020-02-13)
------------------
* Merge pull request `#317 <https://github.com/strands-project/strands_executive/issues/317>`_ from strands-project/fix-316
  Removing buggy code for closing task windows during execution.
* Removing buggy code for closing task windows during execution.
  This closes `#316 <https://github.com/strands-project/strands_executive/issues/316>`_
* removing absolute namespace from battery topic
* Merge pull request `#311 <https://github.com/strands-project/strands_executive/issues/311>`_ from strands-project/ori-kinetic-devel
  Merges in ORI developments from elsewhere
* Blacklist nodes are now excluded from policy execution using the LTL spec.
* Blacklisted nodes are included as part of MDP LTL spec
* Merge remote-tracking branch 'ori/kinetic-devel' into working-kinetic-devel
  Adding in ori developments from last few months.
* Merge pull request `#308 <https://github.com/strands-project/strands_executive/issues/308>`_ from bfalacerda/kinetic-devel
  namespace sorting and params to adapt door and action exec behaviour
* params to ignore door configs and execute actions on the spot
* Merge branch 'kinetic-devel' of https://github.com/strands-project/strands_executive into kinetic-devel
* Added sim_time default value
* Added more policy information into the schedule output
* Added default for parameter
* more namespace updates
* edits for namespaces
* big code clean: remove old (simplified) mdp nodes and scheduler.
* remove uneeded multi robot launch file - one cane use ROS_NAMESPACE=ns instead
* Merge branch 'kinetic-devel' of https://bitbucket.org/oxfordroboticsinstitute/strands_executive into kinetic-devel
* changes for multi-robot
* More informative log message.
* Publishing execution status for easier interaction
* Merge branch 'kinetic-devel' of https://github.com/strands-project/strands_executive into kinetic-devel
* Contributors: Bruno Lacerda, Nick Hawes

1.2.4 (2018-11-07)
------------------
* Merge pull request `#305 <https://github.com/strands-project/strands_executive/issues/305>`_ from francescodelduchetto/pull-req
  cancel_active_task service result message has a boolean to acknowledg…
* cancel_active_task service result message has a boolean to acknowledge the termination of the task
* merge
* Updated examples and output
* Contributors: Nick Hawes, francescodelduchetto

1.2.3 (2018-08-08)
------------------
* added default value for sim time
* Updates to deal with simulated time more directly (since moving to stage), which includes coping with negative times.
* Contributors: Nick Hawes

1.2.2 (2017-12-09)
------------------
* kinetic-1.2.1
* updated changelogs
* Contributors: Nick Hawes

1.2.0 (2017-09-03)
------------------

1.1.2 (2017-08-31)
------------------

1.1.1 (2017-08-31)
------------------

1.1.0 (2017-08-24)
------------------
* made routine more conservative with out of range tasks
* stop dropping tasks when robot fails navigation
* Switched recalled to preempted on task time-outs
* typo
* correct indent
* log adding of tasks to the routine
* add params for doors
* Fixed bug which curtailed execution of time-critical tasks
* Tidied up initialisation.
* Added summary script
* Added script to make task events unique from a previous db
* Added approach for db task id persistence
* Added locks to get ids method
* Added a service to get new task ids for use in other processes.
* Monitor exeuction for overtime. Kill if over an hour.
* is_interruptible flag is respected for MdpTasks
  Also refactored a bit of the logging code for sanity.
* Made active task check more sefl-contained.
  Also added more inforamtion to logging for ltl tasks.
* Fixed bug trying to log the wrong type of task after cancellation
* adding cossafe task now uses the mdptask format
* added parameter allowable_lateness to launch file
* made allowable_lateness a parameter and ignore probability for time_critical tasks
  closes https://github.com/strands-project/aaf_deployment/issues/382
* Removed duplicate lock release.
  Fixes `#284 <https://github.com/strands-project/strands_executive/issues/284>`_
* Added option to terminate execution of a batch of task when the lowest time window has passed.
  To use this option add `close_windows:=true` to the command of launching the system, e.g.
  ```
  roslaunch --wait task_executor mdp-executor.launch close_windows:=true
  ```
* Using expected nav time before window opens.
  This fixes `#281 <https://github.com/strands-project/strands_executive/issues/281>`_
* Get daily_start into datetime.datetime format
* Fix incorrect reference to self.daily end in DailyRoutine
* Dropping normal tasks given expected close of window.
  Also added more detail to task status output.
* Setting default expected duration.
* Prevents time-critical tasks from interrupting on-demand tasks.
* Adds service to demand co-safe tasks.
  This has been tested on local examples, but could use some more extreme tests.
  This closes `#271 <https://github.com/strands-project/strands_executive/issues/271>`_
* Removed some of the longer tests.
  This is in order to let CI testing complete in a sane time.
* Added new combined sort criterion for choosing next tasks.
  The new combined sort criterion is priority*p(success)/expected_time (i.e. reward per unit time) as requested in `#272 <https://github.com/strands-project/strands_executive/issues/272>`_. The old sort criterion criterio is still the default behaviour. To use the new sorting, add `combined_sort:=true` when launching the executive launch file.
  This closes `#272 <https://github.com/strands-project/strands_executive/issues/272>`_
* Bug fixes.
* Added locks to get ids method
* Added a service to get new task ids for use in other processes.
* expected_duration is now being used in mdp executor.
  If a task does not have expected_duration set, max_duration is used instead.
* Added new combined sort criterion for choosing next tasks.
  The new combined sort criterion is priority*p(success)/expected_time (i.e. reward per unit time) as requested in `#272 <https://github.com/strands-project/strands_executive/issues/272>`_. The old sort criterion criterio is still the default behaviour. To use the new sorting, add `combined_sort:=true` when launching the executive launch file.
  This closes `#272 <https://github.com/strands-project/strands_executive/issues/272>`_
* Bug fixes.
* Added locks to get ids method
* Added a service to get new task ids for use in other processes.
* expected_duration is now being used in mdp executor.
  If a task does not have expected_duration set, max_duration is used instead.
* First pass on ability to add "mdp tasks" (i.e. domain spec and some constraints) to the executor.
  This is for `#263 <https://github.com/strands-project/strands_executive/issues/263>`_
* Now using travel time on its own to calculate travel time to start of time-critical task.
* Updates to analysis tools for AAF Y3.
* Printing expected time.
* Dealing with failures a little better, and adding more logging on task insertion
* Dropping time critical too.
* More fixiin
* Now checking against correct variables.
* This makes the routine tougher about out of window tasks.
* Merge branch 'indigo-devel' of https://github.com/strands-project/strands_executive into indigo-devel
* Tasks which are disallowed by the provided tasks_allowed function are now delayed rather than dropped. This means they will be checked until they pass  now + max_duration > end_before
* Re-publishing schedule after clear.
* Merge branch 'indigo-devel' of https://github.com/hawesie/strands_executive into indigo-devel
* Interrupt current task when needs to start time critical
* Contributors: Bruno Lacerda, Marc Hanheide, Michal Staniaszek, Nick Hawes

1.0.6 (2016-06-06)
------------------

1.0.5 (2016-06-06)
------------------

1.0.4 (2016-06-06)
------------------

1.0.3 (2016-06-06)
------------------

1.0.2 (2016-06-06)
------------------

1.0.1 (2016-06-06)
------------------
* Creating a utility function for multi-wp definitions.
* Updated map.
* Updated test to use new tsc top map.
* Contributors: Nick Hawes

1.0.0 (2016-05-29)
------------------
* Preventing empty waypoint from being added to the list.
* Updates to schedule status publishing and printing, mainly for mdp executor.
* Added header field to schedule to allow sync of multiple topics
* Now only publishing active tasks on current_schedule. all_tasks still contains all tasks including those in current_schedule.
* Only interrupting once
* Testing for time critical in execution wait rather than feedback. This allows ongoing tasks to be terminated, not just navigation.
* Checking for itnerruptibility before preempting.
* Added option for clearing all tasks to bypass interruptibility
* Publishing all tasks.
* Not letting clear service clear when an interruptible task is active.
* Exposing wait interruptibility param.
* Added ability to perform a task at one of a list of nodes.
  start_node_id list to be a waypoint list separated with precisely ' | ' no variation on spaces etc.
* Exception handling in routine
* Daealing with time critical tasks in routine
* LTL tasks are now included in the active batch and therefore published as part of the schedule.
  This closes `#234 <https://github.com/strands-project/strands_executive/issues/234>`_
* Sorting by priority as primary key then probability
* Added ability to add extra tasks to live routine.
* Added sorting by end time too
* Sorting by priority as primary key then probability
* Change LTL formula writing to allow reasoning on progression
* Fixed crash on task demand.
* Change LTL formula writing to allow reasoning on progression
* Fixed crash on task demand.
* Removed unnecessary publisher
* Merging published dropped tasks changes into new main branch.
* This adds support for LTL formula execution via the mdp_task_executor.
  The current constraint is that the execution of these tasks is not monitored for completion. This means that if one is interrupted for whatever reason, then they are not retried.
  LTL tasks are specified using the Task's action field, e.g.
  ```
  ltl_task = Task(action='(F "WayPoint4")')
  ```
  the start_after and end_before fields are respected for scheduling, but max_duration is ignored.
* Using execution time to monitor execution.
* adding duration back in to example
* Handling tasks without times more directly
* Changes for executors to propagate active task changes.
  Also changes to move towards monitoring execution time in the mdp_task_executor. In addition this version checks a wider range of tasks for execution.
* Set up a test set that should hopefully pass.
* Changed the base exectur active_task field to be active_tasks list to suit mdp_task_executor.
  This change has been propogated back through the scheduled_task_executor but needs more testing on real tasks.
* Implemented clear_schedule
  This also adds an important update to pause_execution. By waiting for the active batch to become empty we are avoiding exiting the pause method before execution has really stopped.
* Replacing forward slashes with underscores in task names to fix `#14 <https://github.com/strands-project/strands_executive/issues/14>`_
* Approximate schedule being published.
* On-demand tasks and logging updated
* On-demand tasks in and tested
* Updated test file
* Pause/restart fixed, tested and working.
* MDP executor debugged and short-term tested with normal and time-critical tasks.
  This is is still just responding to add_tasks and set_execution_status services, but should be reasonably robust.
  Ready for robot testing.
* Changed to correct door checking then pass action.
* Added mdp test to cmake file
* Automated testing updated.
  The tests are now less strict, but do run well enough to actually catch possible execution-time failures.
* Removed constants from MdpAction, using ones from Task instead so they are directly compatible for automatic conversion.
  This necessitated added STRING_TYPE to the Task msg to keep @bfalacerda happy for completeness.
* Adding exceutor back to launch file.
* First version to cover both time critical and normal tasks. Needs extensive testing.
* Basic executor working.
* Added time window to example.
* Added SortedCollection class
* Initial mdp exec setup
* Excution will now be interrupted after action execution if this is necessary to start a time-critical task.
* Change to how time prediction is done for time-critical tasks.
  Now we update at the start of batch selection time, to only use the current location of the robot.
  Next up, need to check when moving too.
* Now uses probability to order tasks then selects on time.
* Using execution time to monitor execution.
* adding duration back in to example
* Handling tasks without times more directly
* Added try/catch to main threads to prevent exiting.
* Added explanations for dropped tasks.
* A first attempt at publishing when a task is dropped by the executor.
  This is for `#217 <https://github.com/strands-project/strands_executive/issues/217>`_
* Changes for executors to propagate active task changes.
  Also changes to move towards monitoring execution time in the mdp_task_executor. In addition this version checks a wider range of tasks for execution.
* Set up a test set that should hopefully pass.
* Changed the base exectur active_task field to be active_tasks list to suit mdp_task_executor.
  This change has been propogated back through the scheduled_task_executor but needs more testing on real tasks.
* Implemented clear_schedule
  This also adds an important update to pause_execution. By waiting for the active batch to become empty we are avoiding exiting the pause method before execution has really stopped.
* Replacing forward slashes with underscores in task names to fix `#14 <https://github.com/strands-project/strands_executive/issues/14>`_
* Approximate schedule being published.
* On-demand tasks and logging updated
* On-demand tasks in and tested
* Updated test file
* Pause/restart fixed, tested and working.
* MDP executor debugged and short-term tested with normal and time-critical tasks.
  This is is still just responding to add_tasks and set_execution_status services, but should be reasonably robust.
  Ready for robot testing.
* Changed to correct door checking then pass action.
* Added mdp test to cmake file
* Automated testing updated.
  The tests are now less strict, but do run well enough to actually catch possible execution-time failures.
* Removed constants from MdpAction, using ones from Task instead so they are directly compatible for automatic conversion.
  This necessitated added STRING_TYPE to the Task msg to keep @bfalacerda happy for completeness.
* Adding exceutor back to launch file.
* First version to cover both time critical and normal tasks. Needs extensive testing.
* Basic executor working.
* Added time window to example.
* Added SortedCollection class
* Initial mdp exec setup
* Merge pull request `#198 <https://github.com/strands-project/strands_executive/issues/198>`_ from hawesie/node_says_relax
  Automatically set relaxed_nav parameter.
* Moved edge explore functionality out to routine
* Removed exploration tasks as they are dangerous because they don't respect the robot's routine.
* Added the ability to trigger actions to explore edges to improve stats
* Automatically set relaxed_nav parameter.
  The problem we have is that early estimates of navigation durations can be low, causing navigation actions to be killed even when they are working. The relaxed_nav parameter greatly inflates the estimates to prevent this happening, but must be set manually.
  This PR adds a node which automatically sets the value of the relaxed_nav based on the number of nav stats for each edge. This is a rather coarse way of doing it, but given the separation of concerns in the system there is not other way. The better future solution is to get some kind of confidence measure with the estimate.
* Contributors: Bruno Lacerda, Marc Hanheide, Nick Hawes

0.1.2 (2015-08-26)
------------------

0.1.1 (2015-08-26)
------------------
* Fixing install statements so directories are installed not contents
* Contributors: Nick Hawes

0.0.26 (2015-05-13)
-------------------
* Fixing the bug with queue/list of unscheduled tasks
* fixing tiny bug in creating list of throwen tasks
* fixed mismatching of tasks numbers
* Fixing that drop method takes into account all tasks(even the previously scheduled)
* Contributors: Lenka

0.0.25 (2015-05-10)
-------------------
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
