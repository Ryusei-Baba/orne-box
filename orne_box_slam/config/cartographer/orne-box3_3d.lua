include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu",
  published_frame = "base_footprint",
  odom_frame = "odom",
  provide_odom_frame = true, 
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = false,  -- Set to true if odometry is available
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 0.5,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 0.5,
  landmarks_sampling_ratio = 1.,
}

------------------------------------------------------------------------------------------------
-- LOCAL SLAM SETTINGS
------------------------------------------------------------------------------------------------

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 8  -- Adjust to match your CPU

-- TRAJECTORY_BUILDER_3D.use_imu_data = true
TRAJECTORY_BUILDER_3D.min_range = 2.5
TRAJECTORY_BUILDER_3D.max_range = 150
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.15
TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 9.81

TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 5.
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 250
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 8.
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 400

TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = 0.004

TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true

TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 10
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 4
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 40

TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.25
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 15  -- Adjusted for closer-range environments
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.45
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 80
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.hit_probability = 0.55
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.miss_probability = 0.49
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.num_free_space_voxels = 2

------------------------------------------------------------------------------------------------
-- GLOBAL SLAM SETTINGS
------------------------------------------------------------------------------------------------

POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.matcher_rotation_weight = 1.6e3
POSE_GRAPH.matcher_translation_weight = 5e2
POSE_GRAPH.global_sampling_ratio = 0.001

POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.sampling_ratio = 0.005
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.8
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5
POSE_GRAPH.constraint_builder.log_matches = true
POSE_GRAPH.constraint_builder.max_constraint_distance = 100.

POSE_GRAPH.optimization_problem.huber_scale = 3e2
POSE_GRAPH.optimization_problem.acceleration_weight = 750
POSE_GRAPH.optimization_problem.rotation_weight = 140
POSE_GRAPH.optimization_problem.log_solver_summary = false
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 200

POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 10
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 3
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 50

POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.translation_weight = 10.0
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.rotation_weight = 1.0
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.only_optimize_yaw = false
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.use_nonmonotonic_steps = false
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.max_num_iterations = 50
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.num_threads = 1

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_rotational_score = 0.77
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 30.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 10.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(60.)

POSE_GRAPH.global_constraint_search_after_n_seconds = 25.

return options
