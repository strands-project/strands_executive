^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mdp_plan_exec
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
