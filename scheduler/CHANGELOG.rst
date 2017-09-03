^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package scheduler
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


1.2.0 (2017-09-03)
------------------

1.1.2 (2017-08-31)
------------------

1.1.1 (2017-08-31)
------------------

1.1.0 (2017-08-24)
------------------

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

0.1.2 (2015-08-26)
------------------

0.1.1 (2015-08-26)
------------------

0.0.26 (2015-05-13)
-------------------
* remove a file which doesnt belong here
* script to get time durations between waypoints
* Contributors: Lenka

0.0.25 (2015-05-10)
-------------------

0.0.24 (2015-05-05)
-------------------

0.0.23 (2015-04-27)
-------------------

0.0.22 (2015-04-21)
-------------------

0.0.21 (2015-04-15)
-------------------
* change of my email in the scheduler
* Contributors: Lenka

0.0.20 (2015-04-12)
-------------------
* remove text "TODO" for solved lines
* Uncommented priorities in example code
* ScipUser - fixed bug with creating tcons
  Scheduler node - test if times s, e are valid
* added warning when pairs creation fail
* Extension of node with diagnostic comments
* changed criterion to  decide task, now same as in ICRA paper
* fixed bug with writtin to a file
* supporting code if we need to propagate priorities to scheduler
* clean-up verson of scheduler, as on-demand  task are not propagate to scheduler, deteled code which was not needed
* Pairs return now -1 when detecting flaw in input data, -2 when creating flaw themselves
* fixed bug in pairs causing scheduler to fail. Also fix bug with -1 constraint, which was causing that schedule was found for non existing solutions
  extended scheduled_task_executor to throw away tasks with  priorities
* Delete readme.md
* Attempt to have readme files in subfolders
* Uncommented priorities in example code
* ScipUser - fixed bug with creating tcons
  Scheduler node - test if times s, e are valid
* added warning when pairs creation fail
* Extension of node with diagnostic comments
* changed criterion to  decide task, now same as in ICRA paper
* fixed bug with writtin to a file
* supporting code if we need to propagate priorities to scheduler
* clean-up verson of scheduler, as on-demand  task are not propagate to scheduler, deteled code which was not needed
* Pairs return now -1 when detecting flaw in input data, -2 when creating flaw themselves
* fixed bug in pairs causing scheduler to fail. Also fix bug with -1 constraint, which was causing that schedule was found for non existing solutions
  extended scheduled_task_executor to throw away tasks with  priorities
* deleted readme
* Delete readme.md
* Attempt to have readme files in subfolders
* Contributors: Lenka, Lenka Mudrova

0.0.19 (2015-03-31)
-------------------

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
* Added bounds to repeat_every_delta method.
  Also cleaned up scheduled and executor output.
* Contributors: Nick Hawes

0.0.11 (2014-11-18)
-------------------

0.0.10 (2014-11-12)
-------------------
* Added back libscheduler install target.
* Contributors: Nick Hawes

0.0.9 (2014-11-12)
------------------

0.0.8 (2014-11-12)
------------------

0.0.7 (2014-11-07)
------------------

0.0.6 (2014-11-06)
------------------
* Restored missing code.
* Added tests for scheduler.
* Pushed duration service argument through C++ side.
* Contributors: Nick Hawes

0.0.5 (2014-11-01)
------------------
* CMake contains only public files, not those used for testing
* deleted to files, which are used only privately
* Now, tasks are replayed correctly from DB (previously, all tasks were the first one in DB)
* Supporting files, mainly used in experiments.
* Added documentation and deleted methods, which were not used anymore
* Changed pairs_new
* Edited CMakelist
* No backtracking algorithm, creating synthetic data
* creation of synthetic test - including 1,2,3,4 intervals
* still not working, it seems that constraint is not checked againts newly changed
* Added methods editExistingTcons, checkConstraint; not tested
* Added Original version of BC, my system uses also pre variables
* Contributors: Lenka Mudrova

0.0.4 (2014-10-29)
------------------
* Removed prints.
* Removed explicit linking of message_store library which was causing library not found error.
* Contributors: Nick Hawes

0.0.3 (2014-10-29)
------------------
* Changes to build for both single package and repo.
  Having problems with exposing libs created by scipoptsuite which are not actually targets.
* Moving mongodb-store component to top of list for linking errors.
* Added cmake extras to set include directories correctly for scipoptsuite.
* Contributors: Nick Hawes

0.0.1 (2014-10-24)
------------------
* Adding ncurses rosdep and also correct mongodb component order.
* Tidying up package and cmake files.
* Added absolute paths to libraries to ensure that dependent projects get correct linking.
* This simply bulk replaces all ros_datacentre strings to mongodb_store strings inside files and also in file names.
* Added argument for number of instances of scheduler to use.
* updated for better multi support
* Added first task logic to scheduler.
  Also made replay script work with mulitple parallel schedulers.
* Scheduler supports a task, which has the flag NOW
* Fixed issue when no file is given for saving in scheduler
* Added time meaurement for generating constraints and added generation of random schedules
* Adding timeout to scheduler.
* Added 1 minute timeout to scheduler.
* Replay script moves start of task set to 0 to make metrics more sensible.
* Integrated @mudrole1's scheduler changes into node.
* Added file to replay scheduling problems from datacentre
* Added parameter to prevent problem saving.
* Added parameters to scheduler_node and added options, when no parameter is set
* Versions of scheduling algorithm can be chosen now + logging of results to file
* Fixed minor scheduling issues.
  1) Made service calls thread safe.
  2) Fixed order of calls in cancellation
  3) Removed blocking assumption in demand task in scheduler
  4) Changed bounding of tasks based on current execution time.
* Fixed minor scheduling issues.
  1) Made service calls thread safe.
  2) Fixed order of calls in cancellation
  3) Removed blocking assumption in demand task in scheduler
  4) Changed bounding of tasks based on current execution time.
* Reduced calls to mdp time stuff.
* cacheing results, but seems to not be having much effect
* Added logging of scheduling problems to datacentre.
  Inefficient use of space, but faster for CPU (duplicating db content with tasks).
* Added logging of scheduling problems to datacentre.
  Inefficient use of space, but faster for CPU (duplicating db content with tasks).
* Quick fix for empty node ids waiting for `#41 <https://github.com/strands-project/strands_executive/issues/41>`_
* getting scheduler to call right expected time service
* first version of mdp policy execution
* Clarified behaviour around rescheduling after a demand.
  Dropping of out-of-bounds additional tasks are not handled separately to out-of-bounds previously scheduled tasks.
* reverting change in scheduler
* getting example task routines to have proper start and ending points
* Removed pre-variables, enlarged handling of preceding constraints
* Changes for on demand tasks.
  Added service for on-demand tasks.
  Restructued scheduled executor to separate new and old tasks, with the aim to allow this to be used to recover tasks overridden by on-demand requests.
* Adding prism and initial prism-ros interaction
* Delayed execution tasks now working correctly with timer.
* Publishing schedule and handling scheduler fail.
* Set up for just patrolling. Launch file printing to screen sensible amounts.
* Fixed bug, that time t was preceeding start of a task
* Compilation under linux now.
* Looking to add time delays to scheduler and executor, but bug found in scheduler.
* Setting up scheduler tests.
* Running scheduler, receiving back at execution framework.
* Working calls to the scheduler!
* Scheduler C++ node is now called with tasks.
* Adding infrastructure for scheduled execution.
* Compiled from scratch.
* A working compiler with lots of cmake hacks.
* Contributors: Bruno Lacerda, Lenka Mudrova, Nick Hawes
