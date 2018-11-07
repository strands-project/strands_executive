^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package strands_executive_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


1.2.1 (2017-09-15)
------------------

1.2.4 (2018-11-07)
------------------
* Merge pull request `#305 <https://github.com/strands-project/strands_executive/issues/305>`_ from francescodelduchetto/pull-req
  cancel_active_task service result message has a boolean to acknowledg…
* cancel_active_task service result message has a boolean to acknowledge the termination of the task
* merge
* Contributors: Nick Hawes, francescodelduchetto

1.2.3 (2018-08-08)
------------------

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
* add plan and durations to guarantees service
* Contributors: Bruno Lacerda

1.1.0 (2017-08-24)
------------------
* add 'added to routine' constant
* Added approach for db task id persistence
* add MdpTasks
* add max duration to MdpAction
* Added missing service defn.
* Adds service to demand co-safe tasks.
  This has been tested on local examples, but could use some more extreme tests.
  This closes `#271 <https://github.com/strands-project/strands_executive/issues/271>`_
* Added a service to get new task ids for use in other processes.
* fixing bug with getting action outcome based on GoalStatus
* Added a service to get new task ids for use in other processes.
* First pass on ability to add "mdp tasks" (i.e. domain spec and some constraints) to the executor.
  This is for `#263 <https://github.com/strands-project/strands_executive/issues/263>`_
* Added expected time to task definition.
  This closes `#262 <https://github.com/strands-project/strands_executive/issues/262>`_
* Contributors: Bruno Lacerda, Nick Hawes

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

1.0.0 (2016-05-29)
------------------
* Added header field to schedule to allow sync of multiple topics
* Added priority examples to msg
* completing execute policy action feedback
* task guarantees service now needs initial waypoint, and outputs the values for prob, prog, and time
* Changed the base exectur active_task field to be active_tasks list to suit mdp_task_executor.
  This change has been propogated back through the scheduled_task_executor but needs more testing on real tasks.
* On-demand tasks and logging updated
* Removed constants from MdpAction, using ones from Task instead so they are directly compatible for automatic conversion.
  This necessitated added STRING_TYPE to the Task msg to keep @bfalacerda happy for completeness.
* proper naming for the response of the service
  see https://github.com/bfalacerda/strands_executive/issues/6
* Formatting.
* Formatting
* publish feedback when executing non-nav actions
* add service to get estimates for cosafe task
* first version of full planning policy execution extended server
* first version of extended execution
* new action messages
* completing execute policy action feedback
* task guarantees service now needs initial waypoint, and outputs the values for prob, prog, and time
* Changed the base exectur active_task field to be active_tasks list to suit mdp_task_executor.
  This change has been propogated back through the scheduled_task_executor but needs more testing on real tasks.
* On-demand tasks and logging updated
* Removed constants from MdpAction, using ones from Task instead so they are directly compatible for automatic conversion.
  This necessitated added STRING_TYPE to the Task msg to keep @bfalacerda happy for completeness.
* proper naming for the response of the service
  see https://github.com/bfalacerda/strands_executive/issues/6
* Formatting.
* Formatting
* publish feedback when executing non-nav actions
* add service to get estimates for cosafe task
* first version of full planning policy execution extended server
* first version of extended execution
* new action messages
* Contributors: Bruno Lacerda, Nick Hawes

0.1.2 (2015-08-26)
------------------

0.1.1 (2015-08-26)
------------------

0.0.26 (2015-05-13)
-------------------

0.0.25 (2015-05-10)
-------------------

0.0.24 (2015-05-05)
-------------------

0.0.23 (2015-04-27)
-------------------

0.0.22 (2015-04-21)
-------------------
* Added extra constants for routine start/stop/
* Contributors: Nick Hawes

0.0.21 (2015-04-15)
-------------------

0.0.20 (2015-04-12)
-------------------
* this abstract_task_server should have no change, I just messed up my git
* Task message has now priorities field
* this abstract_task_server should have no change, I just messed up my git
* Task message has now priorities field
* Contributors: Lenka

0.0.19 (2015-03-31)
-------------------
* added documentation and made it a *real* abstract class
* Fixing a typo in abstract_task_server
* finished a first draft of the factory from a YAML string
* using yaml to instantiate tasks
* Integrated mdp travel time service.
  The current setup allows and code switch back to top nav if necessary. Tested with both.
  This also fixes a problem in the /mdp_plan_exec/get_expected_travel_times_to_waypoint service where it was expecting a duration for epoch but the service definition was of int.
* removed invalid comments
* moved abstract_task_server into strands_executive_msgs and refactored wait_action
* added an abstract_task_server
* Contributors: Christian Dondrup, Marc Hanheide, Nick Hawes

0.0.18 (2015-03-23)
-------------------

0.0.16 (2014-11-26)
-------------------

0.0.15 (2014-11-23)
-------------------

0.0.14 (2014-11-21)
-------------------

0.0.13 (2014-11-21)
-------------------

0.0.12 (2014-11-20)
-------------------

0.0.11 (2014-11-18)
-------------------

0.0.10 (2014-11-12)
-------------------

0.0.9 (2014-11-12)
------------------

0.0.8 (2014-11-12)
------------------

0.0.7 (2014-11-07)
------------------

0.0.6 (2014-11-06)
------------------
* Added tests for scheduler.
* Pushed duration service argument through C++ side.
* Contributors: Nick Hawes

0.0.5 (2014-11-01)
------------------

0.0.4 (2014-10-29)
------------------
* No change

0.0.3 (2014-10-29)
------------------
* No change

0.0.1 (2014-10-24)
------------------
* Tidying up package and cmake files.
* This simply bulk replaces all ros_datacentre strings to mongodb_store strings inside files and also in file names.
* Added first task logic to scheduler.
  Also made replay script work with mulitple parallel schedulers.
* Added summary printing script
* Fixed minor scheduling issues.
  1) Made service calls thread safe.
  2) Fixed order of calls in cancellation
  3) Removed blocking assumption in demand task in scheduler
  4) Changed bounding of tasks based on current execution time.
* Logging working from state machine now.
* Fixed minor scheduling issues.
  1) Made service calls thread safe.
  2) Fixed order of calls in cancellation
  3) Removed blocking assumption in demand task in scheduler
  4) Changed bounding of tasks based on current execution time.
* Logging working from state machine now.
* Added bool type to task
* Added logging of task event changes to message store.
* Added logging of task event changes to message store.
* allowing larger timeouts for travel time learning action
* adding srv file for special waypoints addition and removal; small bug fixes
* code cleaning and travelling times learning action added
* first version of mdp policy execution
* adding service to update the mdp using the navigation statistics in the db
* On demand tasks working.
  Also added in time and duration types for tasks.
  After a demand the scheduler tries to schedule back in the previously scheduled but unexecuted tasks. If this is not successful then these tasks are dropped. If these are successfully scheduled back in then it also tries to schedule back in the task which was interrupted by the demand. If this is not possible only the interrupted task is dropped.
  Demands can be interrupted by timeout and by subsequent demanded tasks.
* Changes for on demand tasks.
  Added service for on-demand tasks.
  Restructued scheduled executor to separate new and old tasks, with the aim to allow this to be used to recover tasks overridden by on-demand requests.
* Really adding prism
* Adding prism and initial prism-ros interaction
* Delayed execution tasks now working correctly with timer.
* Publishing schedule and handling scheduler fail.
* Trying to get routine adding tested.
* Moved to adding tasks in a batch. Old interface left for compatibility.
* Running scheduler, receiving back at execution framework.
* Working calls to the scheduler!
* Scheduler C++ node is now called with tasks.
* Adding infrastructure for scheduled execution.
* Added int and float arguments to task execution.
* Basic test of FIFO done and working.
  Works from the command line, but can't seem to make the rostest integration work.
* Basic execution flow through abstract and FIFO working.
* Moved test action to task_executor, adding server to provide it.
* Basic node comms working.
* Working basic task creation.
* Adding action and msg types for interaction with task framework.
* Added messages and structure.
* Contributors: Bruno Lacerda, Nick Hawes
