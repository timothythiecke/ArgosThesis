add_library(marching_loop_functions MODULE 
  marching_loop_functions.h
  marching_qt_user_functions.h
  marching_loop_functions.cpp
  marching_qt_user_functions.cpp)

target_link_libraries(marching_loop_functions
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_qtopengl
  ${ARGOS_QTOPENGL_LIBRARIES})
