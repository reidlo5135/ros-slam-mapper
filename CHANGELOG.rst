Changelog
=========

2026-04-02
----------

- Started the ``0.1.0`` development branch:

  - initialized the standalone ``ros-slam-mapper`` repository direction
  - set the initial branch baseline to ``humble/develop/0.1.0``
  - aligned the repository workflow around ``README.md``, ``TODO.md``, ``CHANGELOG.rst``, and ``coding_template.txt``
  - defined the standalone topic namespace direction around ``/slam/*`` instead of ``/amr/*``
  - documented the initial SLAM split as karto-family scan matching front-end plus pose-graph optimization back-end
  - migrated the pasted ``amr_slam_mapper`` business logic into split ``slam_*`` packages
  - localized the standalone bringup parameters onto ``/slam/*`` topics
  - refactored ``slam_mapper`` back into a metapackage-only entry package following the ``amr_navigation`` pattern
  - moved ``map->odom`` ownership in ``slam_pgraph_server`` to a safer back-end-first default policy
  - added ``RCLCPP_INFO`` diagnostics across ``slam_pgraph_server``, ``slam_scan_matcher``, and ``slam_submap_server``
