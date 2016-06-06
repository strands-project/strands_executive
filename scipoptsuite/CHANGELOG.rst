^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package scipoptsuite
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

0.0.22 (2015-04-21)
-------------------

0.0.21 (2015-04-15)
-------------------

0.0.20 (2015-04-12)
-------------------

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

0.0.5 (2014-11-01)
------------------

0.0.4 (2014-10-29)
------------------
* Fixing library type for OS X.
* Making install platform independent.
* Contributors: Nick Hawes


0.0.3 (2014-10-29)
------------------
* Removed readline from scipoptsuite build as we don't have an interactive interface in ROS.
* Installing library files now with link targets as well.
* Changes to build for both single package and repo.
  Having problems with exposing libs created by scipoptsuite which are not actually targets.
* Now correctly fixing build dir problem.
  I was previously hardcoding the prefix in devel space which could either have been strands_executive/scipoptsuite or just scipoptsuite depending on whether this was built as standalone package or a in repo. This can be reflected in the cmake file with the scipoptsuite_BINARY_DIR variable so now I'm using that.
* Added cmake extras to set include directories correctly for scipoptsuite.
* Adding install targets and fixing build prefix for release.
* Contributors: Nick Hawes


0.0.1 (2014-10-24)
------------------
* Adding ncurses rosdep and also correct mongodb component order.
* Added absolute paths to libraries to ensure that dependent projects get correct linking.
* Updated dependencies.
* Added libgmp3-dev depenency. Fixes `#23 <https://github.com/strands-project/strands_executive/issues/23>`_.
* libgmp-dev build_depend
* Compilation under linux now.
* Added md5 hash.
* Adapting build to now apply patches on linux.
* Working calls to the scheduler!
* Added platform switches.
  Now to test on linux vm.
* Compile of scheduler from scratch.
* Linking libraries to platform independent names.
* Compiled from scratch.
* A working compiler with lots of cmake hacks.
* Files installed into correct places, but not through standard ways.
  It's not clear why the cmake install commands are not being executed, but it could because the project doesn't have a proper target.
* First commit of scipoptsuite external build
* Contributors: Chris Burbridge, Nick Hawes
