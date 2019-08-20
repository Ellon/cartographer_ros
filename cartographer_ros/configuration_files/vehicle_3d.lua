-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

-- Parameters inside the 'option' block below are related to
-- Cartographer ROS frontend
options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu",
  published_frame = "vehicle",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 2,
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

-- Some vehicle parameters that are used below
VELODYNE_MAX_RANGE = 100.

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 7

-- Set the max range of scan points
TRAJECTORY_BUILDER_3D.max_range = VELODYNE_MAX_RANGE
-- We use two clouds to build a full scan
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 2
-- Filter input scans in a voxels of 10cm
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.1

-- High resolution clouds will be built from points up o 1/4 of max range
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range = VELODYNE_MAX_RANGE / 4.0
-- Low resolution clouds will be built from points up to max range
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = VELODYNE_MAX_RANGE

-- Make the submap's high res clouds slight larger than scan's high res clouds
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = VELODYNE_MAX_RANGE / 3.0

-- Each submap cell is not observed enough times given the space between scan
-- rings and how fast the vehicle moves. This results in hit cells with low
-- probability, and thus low matching scores. To compensate this we increase
-- this parameter to make the prob raise faster with only few updates.
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.hit_probability = 0.96

-- An attempt to tune ceres_scan_matcher
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_0 = 2. -- default: 1.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_1 = 6. -- default: 6.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 5. -- default: 5.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 2e2 -- default: 4e2

-- Reduce this weight because our IMU angular velocity is a little noisy
POSE_GRAPH.optimization_problem.rotation_weight = 3e3

-- Optimize every two submaps
POSE_GRAPH.optimize_every_n_nodes = 2 * TRAJECTORY_BUILDER_3D.submaps.num_range_data

-- Log everything for debugging 
POSE_GRAPH.log_residual_histograms = true
POSE_GRAPH.constraint_builder.log_matches = true
POSE_GRAPH.optimization_problem.log_solver_summary = true

return options
