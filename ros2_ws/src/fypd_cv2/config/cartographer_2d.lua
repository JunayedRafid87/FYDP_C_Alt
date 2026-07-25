-- Cartographer 2D — FYDP Cv2 (v5.1: IMU-enabled, anti-drift tuning)
--
-- TF ownership is split three ways and nothing overlaps:
--   map  -> odom                  : this node        (provide_odom_frame = false,
--                                                     published_frame    = "odom")
--   odom -> base_link             : odom_gate_node   (RF2O, gated on the IMU state)
--   base_link -> base_link_stabilized -> laser : hardware_interface + robot_state_publisher
--
-- use_odometry consumes /odom, which is the *gated* odometry, not RF2O's raw output.
-- That matters: while the table is stationary the gated odometry reports zero motion,
-- so the pose extrapolator has nothing to integrate and map -> odom holds still.
--
-- v5.1 change: IMU enabled. BNO055 gyroscope gives Cartographer angular velocity
-- for robust pose extrapolation through 180° turns where RF2O loses scan overlap.
-- Loop closure thresholds lowered so Cartographer recognises previously-visited rooms.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- ═══════════════════════════════════════════════════════════════
-- IMU CONFIGURATION
-- ═══════════════════════════════════════════════════════════════
-- BNO055 gyro gives angular velocity for pose extrapolation through
-- 180° turns. Accelerometer gives gravity direction to keep the 2D
-- projection stable.
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0

-- ═══════════════════════════════════════════════════════════════
-- RANGE / SCAN SETTINGS
-- ═══════════════════════════════════════════════════════════════
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.

-- ═══════════════════════════════════════════════════════════════
-- LOCAL SCAN MATCHING — wider search for drift recovery
-- ═══════════════════════════════════════════════════════════════
-- The correlative scan matcher does a brute-force search around the
-- predicted pose. Wider windows let it recover from RF2O drift during
-- sharp turns, at the cost of slightly more CPU.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(30.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- ═══════════════════════════════════════════════════════════════
-- MOTION FILTER — keep fine angular resolution
-- ═══════════════════════════════════════════════════════════════
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

-- ═══════════════════════════════════════════════════════════════
-- SUBMAPS — larger for more context in corridors
-- ═══════════════════════════════════════════════════════════════
-- More scans per submap gives the loop closure matcher more features
-- to recognise revisited areas, especially in featureless corridors.
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 120

-- ═══════════════════════════════════════════════════════════════
-- POSE GRAPH — aggressive loop closure to fix reverse-path drift
-- ═══════════════════════════════════════════════════════════════

-- Optimize the graph more often (default 90, we do 35)
POSE_GRAPH.optimize_every_n_nodes = 35

-- Lower score thresholds so Cartographer is more willing to recognise
-- a room it has visited before, even with accumulated odometry drift.
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6

-- Sample more candidate nodes when looking for loop closure constraints
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3

-- Search wider: a multi-room traversal can accumulate >5m of drift
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.

-- Global loop-closure scan matcher: search across a large area to find
-- the correct position when returning to a previously mapped room
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7

-- Strong weights on accepted loop closures so they actually correct drift
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5

-- Huber loss scale — smaller values make the optimizer more aggressive
-- at correcting outlier-looking poses when a loop closure fires
POSE_GRAPH.optimization_problem.huber_scale = 1e1

return options
