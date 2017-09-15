^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mdp_plan_exec
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.8 (2017-09-14)
------------------

1.0.9 (2017-09-15)
------------------
* Removed stray prism-robots dependency.
* indigo-1.0.8
* Updated changelogs
* Contributors: Nick Hawes

1.0.7 (2017-09-14)
------------------
* Merge pull request `#304 <https://github.com/strands-project/strands_executive/issues/304>`_ from bfalacerda/indigo-devel
  stop doing a full mdp deepcopy
* stop doing a full mdp deepcopy
* Merge pull request `#298 <https://github.com/strands-project/strands_executive/issues/298>`_ from bfalacerda/indigo-devel
  door waiting params
* add params for doors
* allowing to define door wait params via yaml
* Merge pull request `#288 <https://github.com/strands-project/strands_executive/issues/288>`_ from bfalacerda/indigo-devel
  cope with prism timeouts by killing and restarting it
* small bug
* another small bug
* add max duration to MdpAction
* Merge branch 'indigo-devel' of https://github.com/strands-project/strands_executive into indigo-devel
* cope with prism timeouts by killing and restarting it
* Merge pull request `#282 <https://github.com/strands-project/strands_executive/issues/282>`_ from hawesie/nav_time_before_window
  Dealing with windows more actively
* Ensure mdp executor respects is_interruptible flag for action execution.
  Needed for aaf_deployment/`#344 <https://github.com/strands-project/strands_executive/issues/344>`_
* Merge branch 'indigo-devel' of https://github.com/strands-project/strands_executive into gcal_timecritical
  Conflicts:
  gcal_routine/src/gcal_routine/tools.py
* Merge pull request `#273 <https://github.com/strands-project/strands_executive/issues/273>`_ from bfalacerda/indigo-devel
  fixing bug with getting action outcome based on GoalStatus
* fixing bug with getting action outcome based on GoalStatus
* Merge pull request `#264 <https://github.com/strands-project/strands_executive/issues/264>`_ from bfalacerda/indigo-devel
  subscribe to topological map topic, and update MDP structure when a …
* subscribe to tpopological map topic, and update MDP structure when a new topo map is published
* Merge pull request `#261 <https://github.com/strands-project/strands_executive/issues/261>`_ from bfalacerda/indigo-devel
  cope with negative travel expectations
* cope with negative travel expectations
* Edges to be considerered door edges is now a param
* Merge branch 'indigo-devel' of https://github.com/strands-project/strands_executive into indigo-devel
* Contributors: Bruno Lacerda, Marc Hanheide, Nick Hawes

1.0.6 (2016-06-06)
------------------
* made dependency on door_pass explicit
* Contributors: Nick Hawes

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
* Added install for new scripts.
* Making sure remapping is possible for topics.
* Contributors: Nick Hawes

1.0.0 (2016-05-29)
------------------
* include door timeout in time cost for door waiting
* improving preemption
* only look at check_door transitions for door predictions
* other bug
* bug fixes
* integrate door predictions
* other bug
* bug fixes
* stop reading from dfa file
* integrate door predictions
* stop reading from dfa file
* increasing timeout for socket comm
* handling call prism result in policy executor properly
* give some more memory to prism - makes solving lager models slightly faster
* have guarantees estimator build correct topo map mdp
* correct typo
* Merge branch 'full_planning' of https://github.com/bfalacerda/strands_executive into full_planning
  Conflicts:
  mdp_plan_exec/src/mdp_plan_exec/top_map_mdp.py
* back to using install in /opt/prism
* code clean, slight feedback improvement, model fatal nav fails
* proper set of policy init state
* completing execute policy action feedback
* removing unused file
* Added logging and longer delays on timeouts.
* task guarantees service now needs initial waypoint, and outputs the values for prob, prog, and time
* Adding waits for the necessary rosparam.
* proper preemption of non-nav actions
* Fix cancellation flag reset.
* Added locking to estimator to prevent errors due to concurrent access.
* Automated testing updated.
  The tests are now less strict, but do run well enough to actually catch possible execution-time failures.
* Removed constants from MdpAction, using ones from Task instead so they are directly compatible for automatic conversion.
  This necessitated added STRING_TYPE to the Task msg to keep @bfalacerda happy for completeness.
* correct bug with expected guarantees node - forgot to change variable name
* Added waits for the rosparam
* Initial mdp exec setup
* Update mdp_policy_executor_extended.py
  quick edit to address https://github.com/bfalacerda/strands_executive/issues/5
* avoid deleting _da component from policy state definitions
* add door_timeout variable
* make sure model is updated before modelc checking
* correct code for forgetting open doors; add support for door_wait_and_pass action
* publish feedback when executing non-nav actions
* add service to get estimates for cosafe task
* example client
* first version of full planning policy execution extended server
* first version of extended execution
* adding possibility to explicitely model closed doors
* back to using install in /opt/prism
* code clean, slight feedback improvement, model fatal nav fails
* proper set of policy init state
* completing execute policy action feedback
* removing unused file
* Added logging and longer delays on timeouts.
* task guarantees service now needs initial waypoint, and outputs the values for prob, prog, and time
* Adding waits for the necessary rosparam.
* more robust calls to /topological_prediction/predict_edges service.
* proper preemption of non-nav actions
* Fix cancellation flag reset.
* Added locking to estimator to prevent errors due to concurrent access.
* Automated testing updated.
  The tests are now less strict, but do run well enough to actually catch possible execution-time failures.
* Removed constants from MdpAction, using ones from Task instead so they are directly compatible for automatic conversion.
  This necessitated added STRING_TYPE to the Task msg to keep @bfalacerda happy for completeness.
* correct bug with expected guarantees node - forgot to change variable name
* Added waits for the rosparam
* Initial mdp exec setup
* Update mdp_policy_executor_extended.py
  quick edit to address https://github.com/bfalacerda/strands_executive/issues/5
* avoid deleting _da component from policy state definitions
* add door_timeout variable
* make sure model is updated before modelc checking
* Merge branch 'full_planning' of https://github.com/bfalacerda/strands_executive into full_planning
  Conflicts:
  mdp_plan_exec/src/mdp_plan_exec/top_map_mdp.py
* correct code for forgetting open doors; add support for door_wait_and_pass action
* remove unused exec count from transition def
* allowing for execution of policies when starting waypoint is forbidden
* publish feedback when executing non-nav actions
* add service to get estimates for cosafe task
* example client
* first version of full planning policy execution extended server
* first version of extended execution
* making robot stop nav policy when entering a forbidden wp
* adding possibility to explicitely model closed doors
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
* mdp now uses ``topological_map_name `` parameter instead of getting it as an argument
* check for white spaces in node names and edge ids, and raise exception if found
* only advertise services and actions once everything else is initialised
* replace ',' by '.' before trying to convert string to float. fix issue for locales where , is used as the decimal
* Contributors: Bruno Lacerda

0.0.21 (2015-04-15)
-------------------
* check if target waypoint exists before getting expected travel times or executing policies
* ignore waypoints visited after influence area of target waypoint has been reached
* Contributors: Bruno Lacerda

0.0.20 (2015-04-12)
-------------------
* clean unneeded prints
* added extra print for the string created by travel times estimation
* kick typo fix
* added prints to figure out bottleneck
* filling mdp with edge predictions from topological nav
* getting node to die cleanly
* Contributors: Bruno Lacerda

0.0.19 (2015-03-31)
-------------------
* Integrated mdp travel time service.
  The current setup allows and code switch back to top nav if necessary. Tested with both.
  This also fixes a problem in the /mdp_plan_exec/get_expected_travel_times_to_waypoint service where it was expecting a duration for epoch but the service definition was of int.
* Contributors: Nick Hawes

0.0.18 (2015-03-23)
-------------------
* Update README.md
* update README
* test latex rendering
* Contributors: Bruno Lacerda

0.0.17 (2015-03-23)
-------------------
* code clean
  better tracking of execution to allow for general co-safe ltl specs
  correct behaviour when robot is already in influence area of target
* code clean + better user feedback on initialisation
* add dependencies
* fixing version and license
* prepare for release
* code clean and adding policy executor node
* proper argument handling
* expected travel times now call fremen
* client class to get special nodes
* initial stuff for the travel time estimator
* adding node to manage forbidden and safe waypoints
* re-adding prism python client
* building top map mdp from the top map obtained via service call
* package skeleton + basic classes
* Contributors: Bruno Lacerda

0.0.16 (2014-11-26)
-------------------

0.0.15 (2014-11-23)
-------------------

0.0.14 (2014-11-21 16:08)
-------------------------

0.0.13 (2014-11-21 00:07)
-------------------------

0.0.12 (2014-11-20)
-------------------

0.0.11 (2014-11-18)
-------------------

0.0.10 (2014-11-12 21:30)
-------------------------

0.0.9 (2014-11-12 20:17)
------------------------

0.0.8 (2014-11-12 19:26)
------------------------

0.0.7 (2014-11-07)
------------------

0.0.6 (2014-11-06)
------------------

0.0.5 (2014-11-01)
------------------

0.0.4 (2014-10-29 21:12)
------------------------

0.0.3 (2014-10-29 10:43)
------------------------

0.0.1 (2014-10-24)
------------------
* Removed mdp_plan_exec as it's not ready for release.
* This simply bulk replaces all ros_datacentre strings to mongodb_store strings inside files and also in file names.
* publishing policy for visualization
* Updating prism-robots which I missed from origin cherry pick.
* more bug fixes
* buf fixes for concurrency handling
  Conflicts:
  mdp_plan_exec/prism_robots
* adding configurable ports and dir for prism manager
* initial code to avoid concurrency issues plus small code cleaning
  Conflicts:
  mdp_plan_exec/scripts/mdp_planner.py
* Changes found on Bob
* Ensuring mdp planner shuts down when asked.
* Merge branch 'sm_executor' of https://github.com/hawesie/strands_executive into sm_executor
  Conflicts:
  mdp_plan_exec/scripts/mdp_planner.py
  task_executor/src/task_executor/base_executor.py
* Ensuring mdp planner shuts down when asked.
* corrected bug on getting expected travel times
* replanning added for unexpected state transitions
* making sure robot gets to the correct pose on goal waypoint
* Merge branch 'hydro-devel' of https://github.com/BFALacerda/strands_executive into hydro-devel
* outputting succeeded immediately when already in goal waypoint and top_nav also outputs succeeded immediately
* Merge branch 'hydro-devel' of https://github.com/BFALacerda/strands_executive into hydro-devel
* making sure the robot doesnt get stuck in nav loops between waypoints
* fixing stupid bug
* Merge branch 'hydro-devel' of https://github.com/BFALacerda/strands_executive into hydro-devel
* handling situations where no message is published in /current_node
* blog post time before title; 4*expected time threshold
* replanning added for unexpected state transitions
* making sure robot gets to the correct pose on goal waypoint
* making sure goals are cancelled down nav pipeline
* fixing preemption mechanism
* making sure there is always at least a small probability of reaching action target nodes
* reads mdp with states labels initial and target at the same time
* small improvement in policy execution;
  updated robbloging
* back to using topological navigation special modes
  for learning and policy execution
* Update mdp_planner.py
  quick fix for current nav bugs
* policy execution now waits until robot gets to the correct pose before outputting succeeded
  removed unneeded variables
* making policy execution reporting execution failure more accurately - stills needs to be done in smarter way
* getting right image topic, for use in the real robot
* adding image to possible blocked area blog entry
* fixing timers
* fixing action preemption; preliminar use of robblog added
* setting apropriate parameters for topological navigation
* small bug fix for unexpected travel time reporting
* file cleaning
* adding srv file for special waypoints addition and removal; small bug fixes
* adding services to add and delete forbidden/safe waypoints
  getting action to also allow either leaving forbidden waypoints asap or navigate to a safe waypoint asap
* code cleaning and travelling times learning action added
* prints warning when edge nav time is more than twice the expected
* first version of mdp policy execution
* handling 'Unknown' final nodes in nav stats
* code cleaning and small bug fix
* changing prism dir
* changing prism-robots
* readding lost header files
* cleaning prism
* adding service to update the mdp using the navigation statistics in the db
* prism updated, big fixes, adding mdp_planner to launch file
* code cleaning
* saving prism files to temp dir
* getting example task routines to have proper start and ending points
* more prism changes
* allowing to change initial state for expected travel times
* prism updates
* updating prism submodule
* getting prism to compile
* adding git submodule for prism
* Removing prism from git tracking
* using nav data to fill mdp probabilities and costs
* Really adding prism
* Contributors: BFALacerda, Bruno Lacerda, Chris Burbridge, Nick Hawes
