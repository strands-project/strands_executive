^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package scipoptsuite
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
