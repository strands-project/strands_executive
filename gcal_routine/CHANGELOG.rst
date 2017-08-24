^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gcal_routine
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2017-08-24)
------------------
* Use requests package instead of urllib to do request
  Fixes `strands-project/g4s_deployment#165 <https://github.com/strands-project/g4s_deployment/issues/165>`_
* Gcal tools add description to yaml instead of processing separately
* Merge branch 'indigo-devel' of https://github.com/strands-project/strands_executive into gcal_timecritical
  Conflicts:
  gcal_routine/src/gcal_routine/tools.py
* fixed a number of bugs in the handling of timecritical for the new scheduler
  all stemming from the "abuse" of end_before = start_after solution...
* Add python-requests to package xml
* Use requests package instead of urllib to do request
  Fixes `strands-project/g4s_deployment#165 <https://github.com/strands-project/g4s_deployment/issues/165>`_
* a first go at submitting time-critical tasks
  this contributes to fixing https://github.com/strands-project/aaf_deployment/issues/372
* Fixed indentation for exception case. Removed forced time-critical change.
* Dealing with failures a little better, and adding more logging on task insertion
* Contributors: Marc Hanheide, Michal Staniaszek, Nick Hawes

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

0.0.25 (2015-05-10)
-------------------

0.0.24 (2015-05-05)
-------------------

0.0.23 (2015-04-27)
-------------------
* added time window in GCal API query, fixes `#194 <https://github.com/strands-project/strands_executive/issues/194>`_
* Contributors: Marc Hanheide

0.0.22 (2015-04-21)
-------------------
* fixed to use UTC
* Contributors: Marc Hanheide (at AAF control PC)

0.0.21 (2015-04-15)
-------------------
* calling the factory method with pre-populated YAML to allows task to know these values already
* Contributors: Marc Hanheide (at AAF control PC)

0.0.20 (2015-04-12)
-------------------
* added documentation in README.md
* added priorities to gcal routine
* Contributors: Marc Hanheide

0.0.19 (2015-03-31)
-------------------
* now uses the top launch file to make the test work...
* better unit testing including scheduler
* gcal_routine now using factory task service
* Contributors: Marc Hanheide

0.0.18 (2015-03-23)
-------------------

0.0.17 (2015-03-23)
-------------------
* bumped version number
* * extended README (documented all parameters)
  * Made all parameters work
* added README
* param had wrong name
* made this work in reality
* added proper use of task templates
* added arguments
* added first stub for task config from file
* completed package.xml and CMakeLists.txt
* added script and tested the runner working
* first fully working version
* * added shoft_to_now for testing
  * made scheduler call optional
* more generic name
* renamed ical to gcal
* Contributors: Marc Hanheide

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
