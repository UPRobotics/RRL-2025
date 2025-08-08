include "map_builder.lua"
include "trajectory_builder.lua"

options.tracking_frame = "base_link"
options.published_frame = "base_link"
options.num_laser_scans = 1
options.num_point_clouds = 0
options.provide_odom_frame = true
options.publish_frame_projected_to_2d = true
options.use_odometry = false
options.use_nav_sat = false
options.use_landmarks = false

options.num_subdivisions_per_laser_scan = 1
options.min_range = 0.2
options.max_range = 25.0 -- Adjust to match your LiDAR's range (LSC-C25CT3-ET)
options.missing_data_ray_length = 5.0

TRAJECTORY_BUILDER_2D.min_z = -0.1
TRAJECTORY_BUILDER_2D.max_z = 0.1
TRAJECTORY_BUILDER_2D.min_range = 0.2
TRAJECTORY_BUILDER_2D.max_range = 25.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.0
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.087 -- ~5 degrees

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

return options