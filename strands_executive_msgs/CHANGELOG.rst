^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package strands_executive_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


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
