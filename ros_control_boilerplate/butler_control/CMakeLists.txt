include_directories(
  include/
)

# Butler Hardware Interface
add_library(butler_hw_interface
  src/butler_hw_interface.cpp
)
target_link_libraries(butler_hw_interface
  generic_hw_interface
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(butler_hw_main src/butler_hw_main.cpp)
target_link_libraries(butler_hw_main
  butler_hw_interface
  generic_hw_control_loop
  ${catkin_LIBRARIES}
)

## Install ------------------------------------------------------------

# Install libraries
install(TARGETS
    butler_hw_interface
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executables
install(TARGETS
    butler_hw_main
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/butler_control/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
