cmake_minimum_required(VERSION 3.5)
project(thor_hardware_interface)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_manager
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories("header/")
include_directories("libs/libmodbus-3.1.10/src/")
link_directories("libs/libmodbus-3.1.10/src/.libs/")

add_library(thor_hardware_interface SHARED src/thor_hardware_interface.cpp)

# ./configure --with-pic --enable-static --enable-shared=no

target_link_libraries(thor_hardware_interface "modbus")

target_compile_features(thor_hardware_interface PUBLIC cxx_std_17)
target_include_directories(thor_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(thor_hardware_interface
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  Boost
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

install(TARGETS thor_hardware_interface
  EXPORT export_thor_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_thor_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

ament_package()
