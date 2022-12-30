# If G2O is available, compile pose graph optimization stuff
if (g2o_FOUND)
  set(SLAM_g2o_sources src/PoseGraphOptimizer.cxx)
  set(g2o_targets g2o::core g2o::types_slam3d g2o::types_slam3d_addons)
  add_definitions(-DUSE_G2O=1)
endif()

# If basalt is available, compile Basalt spline
if (basalt-headers_FOUND)
  set(SLAM_basalt_sources src/BasaltSpline.cxx)
  set(basalt_target basalt-headers)
  add_definitions(-DUSE_BASALT=1)
  message("Lidar SLAM : Basalt was found, compile Basalt Spline")
else()
  add_definitions((-DUSE_BASALT=0))
endif()

# Generate export symbols on Windows to use this lib
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

add_library(LidarSlam
  src/ConfidenceEstimators.cxx
  src/KeypointsMatcher.cxx
  src/LocalOptimizer.cxx
  src/RollingGrid.cxx
  src/ExternalSensorManagers.cxx
  src/Slam.cxx
  src/SpinningSensorKeypointExtractor.cxx
  src/Utilities.cxx
  src/VoxelGrid.cxx
  src/InterpolationModels.cxx
  ${SLAM_g2o_sources}
  ${SLAM_basalt_sources}
)

target_link_libraries(LidarSlam
  PUBLIC
    nanoflann::nanoflann
    ceres
    ${PCL_LIBRARIES}
    ${g2o_targets}
  PRIVATE
    ${Eigen3_target}
    ${OpenMP_target}
    ${basalt_target}

)

# TODO Test library, delete it later
add_executable(test_interpo_algo
  test/rotation_interpo.cxx)

target_link_libraries(test_interpo_algo
  LidarSlam)

target_include_directories(LidarSlam PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

install(TARGETS LidarSlam
        RUNTIME DESTINATION ${SLAM_INSTALL_LIBRARY_DIR}
        LIBRARY DESTINATION ${SLAM_INSTALL_LIBRARY_DIR}
        ARCHIVE DESTINATION ${SLAM_INSTALL_LIBRARY_DIR}
        PUBLIC_HEADER DESTINATION ${SLAM_INSTALL_INCLUDE_DIR}/LidarSlam
        COMPONENT Runtime)