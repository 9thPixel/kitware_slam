set(interfaces)
set(sources)
set(ui_files
  UserInterface/Resources/UI/sqAboutSlamDialog.ui
  UserInterface/Resources/UI/sqPresetDialog.ui
  UserInterface/Resources/UI/sqSlamToolbar.ui
)

set(subdir_sources
  sqAboutSlamAction.cxx
  sqAboutSlamAction.h
  sqAboutSlamDialog.cxx
  sqAboutSlamDialog.h
  sqAddSlamReaction.cxx
  sqAddSlamReaction.h
  sqButtonPausePropertyWidget.cxx
  sqButtonPausePropertyWidget.h
  sqInitializeDialogReaction.cxx
  sqInitializeDialogReaction.h
  sqPresetDialog.cxx
  sqPresetDialog.h
  sqPresetDialogReaction.cxx
  sqPresetDialogReaction.h
  sqSlamManager.cxx
  sqSlamManager.h
  sqSlamStarter.cxx
  sqSlamStarter.h
  sqSlamToolbar.cxx
  sqSlamToolbar.h
  sqSubsetPropertiesDialog.cxx
  sqSubsetPropertiesDialog.h
  sqTriggerSlamCommandReaction.cxx
  sqTriggerSlamCommandReaction.h
)

foreach(filename ${subdir_sources})
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/UserInterface/${filename}"
    "${CMAKE_CURRENT_BINARY_DIR}/${filename}"
    COPYONLY
  )
  list(APPEND sources "${filename}")
endforeach()

paraview_plugin_add_property_widget(
  KIND WIDGET
  TYPE "command_pause_button"
  CLASS_NAME sqButtonPausePropertyWidget
  INTERFACES pause_widget_interfaces
  SOURCES  pause_widget_sources)

paraview_plugin_add_action_group(
  CLASS_NAME sqAboutSlamAction
  GROUP_NAME "MenuBar/Help"
  INTERFACES about_action_interfaces
  SOURCES about_action_sources
)

paraview_plugin_add_auto_start(
  CLASS_NAME sqSlamStarter
  STARTUP onStartup
  SHUTDOWN onShutdown
  INTERFACES autostart_interface
  SOURCES autostart_sources)

paraview_plugin_add_toolbar(
  CLASS_NAME sqSlamToolbar
  INTERFACES toolbar_interfaces
  SOURCES toolbar_sources)

list(APPEND interfaces
  ${autostart_interface}
  ${pause_widget_interfaces}
  ${about_action_interfaces}
  ${toolbar_interfaces}
)
list(APPEND sources
  ${autostart_sources}
  ${pause_widget_sources}
  ${about_action_sources}
  ${toolbar_sources}
)

set(qrc_files
  UserInterface/Resources/sqResources.qrc
)

list(APPEND CMAKE_AUTOUIC_SEARCH_PATHS
  "${CMAKE_CURRENT_SOURCE_DIR}/UserInterface/Resources/UI")

# Create Plugin
set(CMAKE_PROJECT_VERSION ${LidarSlam_VERSION})
paraview_add_plugin(LidarSlamPlugin
  REQUIRED_ON_CLIENT
  REQUIRED_ON_SERVER
  VERSION ${CMAKE_PROJECT_VERSION}
  UI_RESOURCES ${qrc_files}
  UI_FILES ${ui_files}
  UI_INTERFACES ${interfaces}
  SOURCES ${sources}
  MODULES
    vtkLidarSlam
    vtkSlamUtilities
  MODULE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/vtkLidarSlam/vtk.module"
    "${CMAKE_CURRENT_SOURCE_DIR}/vtkSlamUtilities/vtk.module"
  DOCUMENTATION_DIR "${CMAKE_CURRENT_SOURCE_DIR}/doc/"
)

# Libraries required by UserInterface code
set(required_modules
  ParaView::pqApplicationComponents
  ParaView::pqComponents
  ParaView::pqCore
  ParaView::RemotingServerManager
  VTK::RenderingCore
)

target_link_libraries(LidarSlamPlugin PRIVATE ${required_modules})
