^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mbf_abstract_nav
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.5 (2019-10-11)
------------------
* Update goal pose on replanning, so the feedback remains consistent
* Fix: Reset oscillation timer after executing a recovery behavior
* Remove debug log messages
* Do not pass boost functions to abstract server to (de)activate costmaps.
  Run instead abstract methods (possibly) overridden in the costmap server,
  all costmap-related handling refactored to a new CostmapWrapper class
* On controller execution, check that local costmap is current
* On move_base action, use MoveBaseResult constant to fill outcome in case of oscilation

0.2.4 (2019-06-16)
------------------
* Reduce log verbosity by combining lines and using more DEBUG
* Concurrency container refactoring
* Prevent LOST goals when replanning
* Set as canceled, when goals are preempted by a new plan
* move setAccepted to abstract action
* moved listener notification down after setVelocity
* fix: Correctly fill in the ExePathResult fields
* Fix controller_patience when controller_max_retries is -1
* Change current_twist for last_cmd_vel on exe_path/feedback
* Replace recursive mutexes with normal ones when not needed
* Give feedback with outcome and message for success and error cases from the plugin.

0.2.3 (2018-11-14)
------------------
* Do not publish path from MBF
* Single publisher for controller execution objects
* Ignore max_retries if value is negative and patience if 0
* Avoid annoying INFO log msg on recovery

0.2.2 (2018-10-10)
------------------
* Add outcome and message to the action's feedback in ExePath and MoveBase

0.2.1 (2018-10-03)
------------------
* Fix memory leak
* Fix uninitialized value for cost
* Make MBF melodic and indigo compatible
* Fix GoalHandle references bug in callbacks

0.2.0 (2018-09-11)
------------------
* Update copyright and 3-clause-BSD license
* Concurrency for planners, controllers and recovery behaviors
* New class structure, allowing multiple executoin instances
* Fixes minor bugs

0.1.0 (2018-03-22)
------------------
* First release of move_base_flex for kinetic and lunar
