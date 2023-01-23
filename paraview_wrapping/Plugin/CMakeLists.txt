set(interfaces)
set(sources
  pqSlamManager.cxx
  pqSlamManager.h)

paraview_plugin_add_auto_start(
  CLASS_NAME pqSlamManager
  STARTUP onStartup
  SHUTDOWN onShutdown
  INTERFACES autostart_interface
  SOURCES autostart_sources)
list(APPEND interfaces
  ${autostart_interface})
list(APPEND sources
  ${autostart_sources})

# Create Plugin
set(CMAKE_PROJECT_VERSION ${LidarSlam_VERSION})
paraview_add_plugin(LidarSlamPlugin
  REQUIRED_ON_CLIENT
  REQUIRED_ON_SERVER
  VERSION ${CMAKE_PROJECT_VERSION}
  UI_INTERFACES ${interfaces}
  SOURCES ${sources}
  MODULES vtkLidarSlam
  MODULE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/vtkLidarSlam/vtk.module"
  DOCUMENTATION_DIR "${CMAKE_CURRENT_SOURCE_DIR}/doc/"
)
