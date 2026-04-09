Changelog
=========

2026-04-10
----------

- Started the ``0.3.0`` follow-up branch:

  - kept ``0.2.1`` pose quality as the working baseline
  - moved the next refactor focus to map quality and scan cleanliness
  - introduced a dedicated ``slam_laser_filter`` package direction for pre-filtered scan input
  - switched the SLAM runtime input contract toward ``/slam/mapper/scan/filtered``

2026-04-09
----------

- Started the ``0.2.3`` follow-up branch:

  - reset the roadmap around ``0.2.1`` as the active pose-estimation baseline
  - discarded ``0.2.2`` as a SLAM-quality reference line
  - moved the next major refactor focus from score-model tuning to map representation / rendering quality
  - rechecked the phase order against `slam_toolbox` concepts such as posed-scan-backed map publication and buffered scan handling

2026-04-06
----------

- Started the ``0.2.1`` follow-up branch:

  - formalized the karto-family front-end observability and coarse-to-fine search phases in ``TODO.md``
  - added detailed scan matcher diagnostics for predicted, coarse, fine, and final scores
  - exposed valid beam counts, occupied cell counts, correction magnitude, and reject reasons from scan matching
  - published throttled front-end matcher diagnostics from ``slam_pgraph_server`` for replayable tuning

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
