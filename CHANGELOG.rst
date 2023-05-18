^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package flexbe_app
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
2.4.0 (2023-05-18)
------------------
* Merge develop branch
* Updates for Melodic and Noetic releases on github.com/FlexBE

2.3.0 (2020-11-19)
------------------
* Merge remote-tracking branch 'origin/feature/python3_support' into develop
* Add support for python3
* Use correct state prefix in autocompletion
  (fix `#65 <https://github.com/FlexBE/flexbe_app/issues/65>`_)
* Prevent module caching in Python parser to fix state reloading
  (see `#61 <https://github.com/FlexBE/flexbe_app/issues/61>`_)
* Sort input-dependent elements for more deterministic code generation
  (see #`team-vigir/flexbe_behavior_engine#111 <https://github.com/team-vigir/flexbe_behavior_engine/issues/111>`_)
* Contributors: Philipp Schillinger

2.2.4 (2020-03-25)
------------------
* Revert nwjs version increment because of Travis failure due to Chromium bug
  (see https://crbug.com/1025266)
* Increment nwjs version after file chooser fix
* Use HTML file chooser instead of Chrome file API for settings import and export
  (see `#52 <https://github.com/FlexBE/flexbe_app/issues/52>`_)
* Improve cache for workspace switching and allow to disable it
* Fix detection of new packages
* Merge branch 'benmaidel-feature/cmd_option_checkbehaviors' into develop
* Always run all behavior check tests and print error message
* Robustify behavior loading for generating behavior check reports
* Close window-related sub-processes also when only closing the window
  (see `team-vigir/flexbe_behavior_engine#105 <https://github.com/team-vigir/flexbe_behavior_engine/issues/105>`_)
* Merge remote-tracking branch 'origin/fix/handle_closed_window' into develop
* add --check-behavior command line option
* Fix support for symbolic references in Python parser
* Robustify Python parser against instantiation failures
  (see `team-vigir/flexbe_behavior_engine#99 <https://github.com/team-vigir/flexbe_behavior_engine/issues/99>`_)
* Close window-related sub-processes also when only closing the window
  (see `team-vigir/flexbe_behavior_engine#105 <https://github.com/team-vigir/flexbe_behavior_engine/issues/105>`_)
* Merge branch 'cheffe112-bugfix-import-config' into develop
* Fix mixed indentation
* reverting nw.js to last known-to-work version
  From nw.js 0.42.4, the import/export configuration functionality throws an "Invalid calling page. This function can't be called from a background page." error. This has been reported for nw.js already on https://github.com/nwjs/nw.js/issues/7349.
  Hence, the import/export functionality is currently unusable with nw.js >0.42.3 as integrated in b93078fd6705445bf6183af40598619243800b1e.
* throw error if importing/exporting configuration crashes
* Explicitly spawn window on center position because default changed
* Allow comment after super-call in regex state parser
  (see `team-vigir/flexbe_behavior_engine#98 <https://github.com/team-vigir/flexbe_behavior_engine/issues/98>`_)
* Check for undefined transition label (fix `team-vigir/flexbe_behavior_engine#100 <https://github.com/team-vigir/flexbe_behavior_engine/issues/100>`_)
* Merge branch 'cheffe112-bugfix-python-parsing' into develop
* Fix mixed indentation
* bugfix: avoid crash during Python state parsing on empty documentation string
  The Python state parsing feature introduced in 4f20227ef900ca32e78414c190fb464d964666e5 used to crash when no docstring is provided in a state definition, hence stalling the parsing procedure. This commit returns the default label "[no documentation]" on a missing docstring.
* Contributors: Benjamin Maidel, Philipp Schillinger, Tobias Doernbach

2.2.3 (2020-01-15)
------------------
* Increment nwjs version
* Fix documentation format warnings (see `#48 <https://github.com/FlexBE/flexbe_app/issues/48>`_)
* Contributors: Philipp Schillinger

2.2.2 (2020-01-11)
------------------
* Add python-based state parser alternative (see `team-vigir/flexbe_behavior_engine#93 <https://github.com/team-vigir/flexbe_behavior_engine/issues/93>`_)
* Implement a quick connect when dropping a state onto a transition
* Add internal dependencies (see `#39 <https://github.com/FlexBE/flexbe_app/issues/39>`_)
* Enable package cache and track update progress
* Clear outcome request on transition and explicit message
* Robustify package parser against malformed package.xml (see `#37 <https://github.com/FlexBE/flexbe_app/issues/37>`_)
* Contributors: Philipp Schillinger

2.2.1 (2019-06-02)
------------------
* Match update of flexbe_ci to xenial
* Add dependency on libnss3 to package.xml (see `#35 <https://github.com/FlexBE/flexbe_app/issues/35>`_)
* Improve support for activities
* Merge remote-tracking branch 'origin/feature/sub_parameters' into develop
* Various minor improvements
* Allow to set parameters of included behaviors
* Merge remote-tracking branch 'origin/master' into develop
* Contributors: Philipp Schillinger

2.1.5 (2019-04-09)
------------------
* Merge remote-tracking branch 'origin/develop'
* Merge pull request `#33 <https://github.com/FlexBE/flexbe_app/issues/33>`_ from henroth/master
  Check that package.xml exists before trying to read it.
* Fix indentation and variable declaration
* Check that package.xml exists before trying to read it. Very old packages/stacks may not have a package.xml, but ros will report them as existing packages
* Contributors: Henry Roth, Philipp Schillinger

2.1.3 (2018-12-19)
------------------
* Merge remote-tracking branch 'origin/develop'
* Add cmake dependency on rostest
* Merge remote-tracking branch 'origin/master' into develop
* Contributors: Philipp Schillinger

2.1.2 (2018-12-18)
------------------
* Merge remote-tracking branch 'origin/develop'
* Switch to curl for nwjs download
* Contributors: Philipp Schillinger

2.1.1 (2018-12-18)
------------------
* Merge remote-tracking branch 'origin/develop'
* Fix #29: Use correct statelib call to open source code
* Contributors: Philipp Schillinger

2.1.0 (2018-12-01)
------------------
* Initial ROS release
* Contributors: Philipp Schillinger

2.0.11 (2018-12-01)
-------------------
* Merge remote-tracking branch 'origin/develop'
* Add shortcut support for sourcing
* Fix `#8 <https://github.com/FlexBE/flexbe_app/issues/8>`_: Correct typo in synthesis feedback
* Fix `#15 <https://github.com/FlexBE/flexbe_app/issues/15>`_: Correctly handle duplicate state class definitions
* Merge remote-tracking branch 'origin/develop'
* Merge remote-tracking branch 'origin/master' into develop
* Hide detailed install output
* Increment nwjs version
* Merge branch 'feature/add_tests' into develop
* Update manifest
* Create .travis.yml
* Set test_report to executable
* Add test routine
* Merge remote-tracking branch 'origin/feature/install_support' into develop
* Fix `#25 <https://github.com/FlexBE/flexbe_app/issues/25>`_: Use python path instead of package path
* Update manifest
* Prevent behavior modifications when loading from install space
* Move package python path out of package parser
* Fixes issue `#24 <https://github.com/FlexBE/flexbe_app/issues/24>`_ using first proposed solution.
* implementing ROS.getPackagePythonPath similarly to IO.PackageParser's getPythonPath
* Use package path for manual section update
* Merge branch 'tu-darmstadt-ros-pkg-master' into feature/install_support
* Adjust catkin install paths for rospack use
* Merge branch 'master' of https://github.com/tu-darmstadt-ros-pkg/flexbe_app into tu-darmstadt-ros-pkg-master
  Conflicts:
  CMakeLists.txt
  bin/run_app
  src/io/io_behaviorloader.js
  src/io/io_packageparser.js
* Parse installed packages (see `#19 <https://github.com/FlexBE/flexbe_app/issues/19>`_)
* Merge pull request `#18 <https://github.com/FlexBE/flexbe_app/issues/18>`_ from meyerj/feature/install-rules
  Add cmake install rules and use rospack to find nw executable
* fix state path to correct generated import statements
* make locating behavior files work in install and devel setups
* make behaviors work in install space
* Add cmake install rules and use rospack to find nw executable
* Fix `#14 <https://github.com/FlexBE/flexbe_app/issues/14>`_: Update state definition only for python files but any event type
* Merge pull request `#13 <https://github.com/FlexBE/flexbe_app/issues/13>`_ from FlexBE/feature/state_update
  Update states when source code changes (see `#10 <https://github.com/FlexBE/flexbe_app/issues/10>`_)
* Update manifest
* Update states when source code changes (see `#10 <https://github.com/FlexBE/flexbe_app/issues/10>`_)
* Contributors: Dorian Scholz, Dustin Gooding, Johannes Meyer, Philipp Schillinger

2.0.10 (2018-11-24)
-------------------
* Merge remote-tracking branch 'origin/develop'
* Contributors: Philipp Schillinger

2.0.6 (2018-03-04)
------------------
* Merge remote-tracking branch 'origin/develop'
* Make behavior name processing more robust (fix `team-vigir/flexbe_behavior_engine#51 <https://github.com/team-vigir/flexbe_behavior_engine/issues/51>`_)
* Update manifest
* Fix `#12 <https://github.com/FlexBE/flexbe_app/issues/12>`_: Improved responsiveness of connecting transitions
* Fix `#9 <https://github.com/FlexBE/flexbe_app/issues/9>`_: Correctly reset transitions to outcomes and add removal
* Remove requirement of keyring access
* Merge remote-tracking branch 'origin/feature/autoinstall' into develop
* Merge remote-tracking branch 'origin/master' into feature/autoinstall
* Install nwjs on running catkin build
* Removed nwjs files and added install to first execution
* Contributors: Philipp Schillinger

2.0.5 (2017-10-01)
------------------
* Several minor additions and fixes
* Update manifest
* Can select to use default values for behavior input keys (see `team-vigir/flexbe_behavior_engine#38 <https://github.com/team-vigir/flexbe_behavior_engine/issues/38>`_)
* Fix `#7 <https://github.com/FlexBE/flexbe_app/issues/7>`_: Whitespace before first state parameter now optional
* Fix `#6 <https://github.com/FlexBE/flexbe_app/issues/6>`_: Compare float value not int for parameter value bounds
* Enable utf-8 encoding in generated behaviors
* Fix `#5 <https://github.com/FlexBE/flexbe_app/issues/5>`_: Negative values for numeric parameters
* Added support for state and behavior packages in editor
* Contributors: Philipp Schillinger

2.0.2 (2017-04-23)
------------------
* Update manifest
* Add button to view state source code
* Fix: use correct attribute to determine drag indicator width
* Fix: stop that states jump to zero if move icon is only clicked
* Fix: creating a new behavior fails when onboard engine is running (see `#4 <https://github.com/FlexBE/flexbe_app/issues/4>`_)
* Fix: creating a new behavior fails without error log (see `#4 <https://github.com/FlexBE/flexbe_app/issues/4>`_)
* Fixed missing yaml import in ROS action client
* Support opening multiple windows
* Only update drawing on outcome request if available (fixes `#2 <https://github.com/FlexBE/flexbe_app/issues/2>`_)
* Contributors: Philipp Schillinger

2.0.1 (2017-02-25)
------------------
* Update manifest
* Fix to avoid placement of new states under container path label
* Fixed function reference for visual update of autonomy level change
* Removed deprecated roslib import
* Contributors: Philipp Schillinger

2.0.0 (2017-01-16)
------------------
* Update README.md
* Made required files executable
* Initial commit of software
* Update README.md
* Initial commit
* Contributors: Philipp Schillinger
