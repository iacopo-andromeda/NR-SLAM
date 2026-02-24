cmake_minimum_required(VERSION 3.23)

find_package(OpenCV 4 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(Armadillo REQUIRED)
find_package(cereal REQUIRED)
find_package(ensmallen REQUIRED)
find_package(mlpack REQUIRED)
find_package(Pangolin REQUIRED)

set(root ${CMAKE_CURRENT_SOURCE_DIR})

set(SLAM_LIB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)

set(NR_SLAM_COMMON_INCLUDES
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/third_party/Sophus
        ${CMAKE_SOURCE_DIR}/third_party/g2o
        ${Pangolin_INCLUDE_DIRS}
        ${ARMADILLO_INCLUDE_DIRS}
        ${CEREAL_INCLUDE_DIR}
        ${ENSMALLEN_INCLUDE_DIR}
        ${MLPACK_INCLUDE_DIRS}
        )

set(NR_SLAM_COMMON_LIBS
        absl::check
        absl::log
        absl::status
        absl::statusor
        absl::die_if_null
        absl::log_flags
        absl::log_globals
        absl::log_initialize
        absl::log_entry
        absl::log_sink
        absl::log_sink_registry
        absl::log_streamer
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        fmt
        ${Pangolin_LIBRARIES}
        ${ARMADILLO_LIBRARIES}
        core
        stuff
        types_sba
        )

function(add_nr_slam_module target_name)
    add_library(${target_name} SHARED ${ARGN})
    target_include_directories(${target_name} PUBLIC ${NR_SLAM_COMMON_INCLUDES})
    target_link_libraries(${target_name} PUBLIC ${NR_SLAM_COMMON_LIBS})
endfunction()

set(CALIBRATION_SOURCES
        ${root}/calibration/kannala_brandt_8.cc
        ${root}/calibration/pin_hole.cc
        ${root}/calibration/camera_model.h
        ${root}/calibration/kannala_brandt_8.h
        ${root}/calibration/pin_hole.h
        )

set(DATASETS_SOURCES
        ${root}/datasets/endomapper.cc
        ${root}/datasets/hamlyn.cc
        ${root}/datasets/simulation.cc
        ${root}/datasets/andromeda.cc
        ${root}/datasets/endomapper.h
        ${root}/datasets/hamlyn.h
        ${root}/datasets/simulation.h
        ${root}/datasets/andromeda.h
        )

set(FEATURES_SOURCES
        ${root}/features/shi_tomasi.cc
        ${root}/features/feature.h
        ${root}/features/shi_tomasi.h
        )

set(MAP_SOURCES
        ${root}/map/frame.cc
        ${root}/map/keyframe.cc
        ${root}/map/map.cc
        ${root}/map/mappoint.cc
        ${root}/map/regularization_graph.cc
        ${root}/map/temporal_buffer.cc
        ${root}/map/frame.h
        ${root}/map/keyframe.h
        ${root}/map/map.h
        ${root}/map/mappoint.h
        ${root}/map/regularization_graph.h
        ${root}/map/temporal_buffer.h
        )

set(MAPPING_SOURCES
        ${root}/mapping/mapping.cc
        ${root}/mapping/mapping.h
        )

set(MASKING_SOURCES
        ${root}/masking/bright_filter.cc
        ${root}/masking/border_filter.cc
        ${root}/masking/masker.cc
        ${root}/masking/predefined_filter.cc
        ${root}/masking/circle_filter.cc
        ${root}/masking/bright_filter.h
        ${root}/masking/border_filter.h
        ${root}/masking/masker.h
        ${root}/masking/predefined_filter.h
        ${root}/masking/circle_filter.h
        )

set(MATCHING_SOURCES
        ${root}/matching/lucas_kanade_tracker.cc
        ${root}/matching/lucas_kanade_tracker.h
        )

set(OPTIMIZATION_SOURCES
        ${root}/optimization/g2o_optimization.cc
        ${root}/optimization/landmark_vertex.cc
        ${root}/optimization/position_regularizer.cc
        ${root}/optimization/position_regularizer_with_deformation.cc
        ${root}/optimization/reprojection_error.cc
        ${root}/optimization/reprojection_error_only_deformation.cc
        ${root}/optimization/reprojection_error_only_pose.cc
        ${root}/optimization/reprojection_error_with_deformation.cc
        ${root}/optimization/spatial_regularizer.cc
        ${root}/optimization/spatial_regularizer_fixed.cc
        ${root}/optimization/spatial_regularizer_with_deformation.cc
        ${root}/optimization/spatial_regularizer_with_observation.cc
        ${root}/optimization/g2o_optimization.h
        ${root}/optimization/landmark_vertex.h
        ${root}/optimization/position_regularizer.h
        ${root}/optimization/position_regularizer_with_deformation.h
        ${root}/optimization/reprojection_error.h
        ${root}/optimization/reprojection_error_only_deformation.h
        ${root}/optimization/reprojection_error_only_pose.h
        ${root}/optimization/reprojection_error_with_deformation.h
        ${root}/optimization/spatial_regularizer.h
        ${root}/optimization/spatial_regularizer_fixed.h
        ${root}/optimization/spatial_regularizer_with_deformation.h
        )

set(SLAM_SOURCES
        ${root}/SLAM/settings.cc
        ${root}/SLAM/system.cc
        ${root}/SLAM/settings.h
        ${root}/SLAM/system.h
        )

set(STEREO_SOURCES
        ${root}/stereo/stereo_lucas_kanade.cc
        ${root}/stereo/stereo_pattern_matching.cc
        ${root}/stereo/stereo_lucas_kanade.h
        ${root}/stereo/stereo_matcher.h
        ${root}/stereo/stereo_pattern_matching.h
        )

set(TRACKING_SOURCES
        ${root}/tracking/essential_matrix_initialization.cc
        ${root}/tracking/monocular_map_initializer.cc
        ${root}/tracking/tracking.cc
        ${root}/tracking/essential_matrix_initialization.h
        ${root}/tracking/monocular_map_initializer.h
        ${root}/tracking/tracking.h
        )

set(UTILITIES_SOURCES
        ${root}/utilities/dbscan.cc
        ${root}/utilities/frame_evaluator.cc
        ${root}/utilities/geometry_toolbox.cc
        ${root}/utilities/landmark_status.cc
        ${root}/utilities/statistics_toolbox.cc
        ${root}/utilities/time_profiler.cc
        ${root}/utilities/types_conversions.cc
        ${root}/utilities/dbscan.h
        ${root}/utilities/frame_evaluator.h
        ${root}/utilities/geometry_toolbox.h
        ${root}/utilities/landmark_status.h
        ${root}/utilities/statistics_toolbox.h
        ${root}/utilities/time_profiler.h
        ${root}/utilities/types_conversions.h
        )

set(VISUALIZATION_SOURCES
        ${root}/visualization/color_factory.cc
        ${root}/visualization/image_visualizer.cc
        ${root}/visualization/map_visualizer.cc
        ${root}/visualization/color_factory.h
        ${root}/visualization/image_visualizer.h
        ${root}/visualization/map_visualizer.h
        )

add_nr_slam_module(nr_slam_calibration ${CALIBRATION_SOURCES})
add_nr_slam_module(nr_slam_datasets ${DATASETS_SOURCES})
add_nr_slam_module(nr_slam_features ${FEATURES_SOURCES})
add_nr_slam_module(nr_slam_map ${MAP_SOURCES})
add_nr_slam_module(nr_slam_mapping ${MAPPING_SOURCES})
add_nr_slam_module(nr_slam_masking ${MASKING_SOURCES})
add_nr_slam_module(nr_slam_matching ${MATCHING_SOURCES})
add_nr_slam_module(nr_slam_optimization ${OPTIMIZATION_SOURCES})
add_nr_slam_module(nr_slam_slam ${SLAM_SOURCES})
add_nr_slam_module(nr_slam_stereo ${STEREO_SOURCES})
add_nr_slam_module(nr_slam_tracking ${TRACKING_SOURCES})
add_nr_slam_module(nr_slam_utilities ${UTILITIES_SOURCES})
add_nr_slam_module(nr_slam_visualization ${VISUALIZATION_SOURCES})

set(NR_SLAM_MODULE_LIBS
        nr_slam_calibration
        nr_slam_datasets
        nr_slam_features
        nr_slam_map
        nr_slam_mapping
        nr_slam_masking
        nr_slam_matching
        nr_slam_optimization
        nr_slam_slam
        nr_slam_stereo
        nr_slam_tracking
        nr_slam_utilities
        nr_slam_visualization
        PARENT_SCOPE)