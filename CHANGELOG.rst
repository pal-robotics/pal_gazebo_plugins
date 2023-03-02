^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pal_gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.3 (2016-07-11)
------------------
* Changed set force to set angle in gazebo underactuated finger plugin
* Added gazebo world odometry that outputs rpy
* Contributors: Hilario Tome

Forthcoming
-----------
* Merge branch 'fix_warns' into 'humble-devel'
  Fix warnings
  See merge request common/pal_gazebo_plugins!24
* compare gazebo times instead of doubles
* use auto instead of string
* Contributors: Jordan Palacios, Noel Jimenez

4.0.3 (2023-02-24)
------------------
* Merge branch 'gazebo_dev_libraries' into 'humble-devel'
  Switch to ament_cmake_auto
  See merge request common/pal_gazebo_plugins!23
* switch to ament_cmake_auto
* Contributors: Jordan Palacios, Noel Jimenez

4.0.2 (2023-02-14)
------------------
* Merge branch 'fix_header' into 'humble-devel'
  Update obsolete header
  See merge request common/pal_gazebo_plugins!22
* update obsolete header
* Contributors: Jordan Palacios, Noel Jimenez

4.0.1 (2023-02-10)
------------------
* Merge branch 'fix_dependency' into 'humble-devel'
  change gazebo dependency to gazebo_dev
  See merge request common/pal_gazebo_plugins!21
* change gazebo dependency to gazebo_dev
* Contributors: Jordan Palacios, Noel Jimenez

4.0.0 (2023-02-08)
------------------
* Merge branch 'linters' into 'humble-devel'
  Cleanup and linters
  See merge request common/pal_gazebo_plugins!20
* update maintainers
* update copyright
* add license
* add CONTRIBUTING.md
* linters
* rm ros1 plugins
* Merge branch 'fix_underactuated_finger' into 'humble-devel'
  Fix underactuated finger
  See merge request common/pal_gazebo_plugins!19
* syntax fixes
* fix seg fault when using pointers
* fix logging errors
* add missing dependencies
* Merge branch 'comment_gazebo_underactuated_finger' into 'ros2'
  comment gazebo_underactuated_finger plugin
  See merge request common/pal_gazebo_plugins!18
* comment gazebo_underactuated_finger plugin
* Merge branch 'foxy_obstacle_avoidance' into 'ros2'
  placeholders and change to gazebo node
  See merge request common/pal_gazebo_plugins!16
* Merge branch 'world-odometry-migration' into 'ros2'
  World odometry migration to ros2
  See merge request common/pal_gazebo_plugins!15
* create gazebo world odometry plugin based on p3d
* set parameter range without steps
* placeholders and change to gazebo node
* World odometry migration to ros2
* Merge branch 'collisions_plugin_ros2' into 'ros2'
  ros2 collisions plugin
  See merge request common/pal_gazebo_plugins!14
* ros2 collisions plugin
* Merge branch 'ferrum-devel' into ros2
* add format tests
* Merge branch 'ros2' of gitlab:common/pal_gazebo_plugins into ros2
* change installation of .h to .hpp
* Merge branch 'ros2' of gitlab:common/pal_gazebo_plugins into ros2
* change installations .h to .hpp
* change .h to .hpp
* change Duration format to ros2
* Merge branch 'ros2' of gitlab:common/pal_gazebo_plugins into ros2
* shared library
* ament package
* package.xml
* shared_ptr error
* compilation repair
* ros2 migration initial commit
* Contributors: Jordan Palacios, Noel Jimenez, Noel Jimenez Garcia, cescfolch, victor

2.0.3 (2021-08-10)
------------------
* Merge branch 'fix_odometry_link' into 'ferrum-devel'
  fix odometry link
  See merge request common/pal_gazebo_plugins!13
* fix odometry link
* Merge branch 'navigation_dynamic_obstacles_test' into 'ferrum-devel'
  update rate
  See merge request common/pal_gazebo_plugins!10
* update rate
* Contributors: Noel Jimenez Garcia, victor

2.0.2 (2021-07-21)
------------------
* Merge branch 'navigation_dynamic_obstacles_test' into 'ferrum-devel'
  rename collisions plugin
  See merge request common/pal_gazebo_plugins!9
* rename collisions plugin
* Merge branch 'navigation_dynamic_obstacles_test' into 'ferrum-devel'
  Navigation dynamic obstacles test
  See merge request common/pal_gazebo_plugins!8
* solve issues
* format code
* add header and activate contacts
* gazebo collisions plugin
* Contributors: Noel Jimenez Garcia, victor

2.0.1 (2021-06-28)
------------------
* Merge branch 'expand-world-odometry' into 'ferrum-devel'
  Add offsets to world odometry and correct odom publishing errors
  See merge request common/pal_gazebo_plugins!7
* Add offsets to world odometry and correct odom publishing errors
* Contributors: Victor Lopez, victor

2.0.0 (2019-09-10)
------------------
* Fixed shadowed variables
* added gazebo 9 API changes
* Contributors: Jordan Palacios, Sai Kishor Kothakota

1.1.9 (2019-05-20)
------------------
* Merge branch 'license-refactor' into 'erbium-devel'
  Update pal license
  See merge request common/pal_gazebo_plugins!5
* Update PAL licenses
* Contributors: Victor Lopez

1.1.8 (2018-05-17)
------------------
* Rotate object position using target link pose
* Contributors: Victor Lopez

1.1.7 (2018-03-29)
------------------
* Actively try to make the attachment on each world update loop
* Merge branch 'gazebo-attachment-plugin' into 'erbium-devel'
  Add gazebo_attachment plugin
  See merge request common/pal_gazebo_plugins!4
* Improve error checking of gazebo_attachment
* Add gazebo_attachment plugin
* Contributors: Hilario Tome, Victor Lopez

1.1.6 (2018-03-08)
------------------
* Merge branch 'titanium_simulation_issue' into 'erbium-devel'
  Control in effort when PID's are set. Otherwise control in position
  See merge request common/pal_gazebo_plugins!3
* Control in effort when PID's are set. Otherwise control in position
* Contributors: Adria Roig, Hilario Tome

1.1.5 (2018-01-30)
------------------
* added gazebo_ros depend
* Merge branch 'gazebo7' into erbium-devel
* Merge branch 'allow-params-on-namespace' into 'dubnium-devel'
  Allow gains to be pushed onto a namespace
  See merge request !2
* Allow gains to be pushed onto a namespace
* Added gazebo7 support
* Contributors: Hilario Tome, Hillario Tome, davidfernandez

1.1.4 (2016-10-14)
------------------
* Added missing depend
* Merge branch 'dubnium-devel' of gitlab:common/pal_gazebo_plugins into dubnium-devel
* Removed hardcoded base name in gazebo world odometry
* Changed world odom to use quaternion intstead of rpy
* 1.1.3
* Updated changelog
* Changed set force to set angle in gazebo underactuated finger plugin
* Added gazebo world odometry that outputs rpy
* Contributors: Hilario Tome

1.1.2 (2016-04-18)
------------------
* Merge branch 'finget_plugin_pid' into 'dubnium-devel'
  Finget plugin pid
  See merge request !1
* Being a bit more verbose on the initialization of the pluginç
* Cleanup
* Changed from set position to pid in finger plugin
* Remove wrongly placed link flag in GAZEBO_LIBRARIES
* Contributors: Hilario Tome, Sam Pfeiffer, Victor Lopez

1.1.1 (2016-04-15)
------------------
* Remove gazebo_ros_range, already merged into upstream gazebo_plugins
* Contributors: Victor Lopez

1.1.0 (2015-06-05)
------------------
* Remove Paul from maintainer
* Fix catkin_package dependency
* Add build and run depends on gazebo
* Add generic underactuated finger plugin for gazebo simulation
* Contributors: Luca Marchionni

1.0.1 (2014-11-17)
------------------
* Added plugin for harnessing the robot in simulation
* Adding plugin for wifi access point simulation in gazebo
* Simple plugin to move underactuated finger joints
* Deprecate PalModelPlugin
* Add launch files and run_gzserver script
* Catkinize, remove parts already in hydro
* Update to newer sdf API
* Move common code from robot-specific repos.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Enrique Fernandez, Luca Marchionni, Paul Mathieu
