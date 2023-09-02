cmake_minimum_required(VERSION 3.5)
project(theta_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost 1.53 REQUIRED system thread timer)
find_package(libuvc REQUIRED)

find_package(PkgConfig)
pkg_check_modules(GSTREAMER_1_0 gstreamer-1.0)
pkg_check_modules(GSTREAMER_1_0_APP gstreamer-app-1.0)
pkg_check_modules(LibUVC libuvc)

include_directories(
  include
  3rd/libuvc-theta-sample/gst
  ${OpenCV_INCLUDE_DIRS}
  ${GSTREAMER_1_0_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  std_srvs
  rclcpp_components
)

# Add library and dependencies
add_library(theta_driver_lib SHARED
  src/theta_driver_lib.cpp 
  3rd/libuvc-theta-sample/gst/thetauvc.c)
rclcpp_components_register_node(theta_driver_lib PLUGIN "theta_driver::ThetaDriver" EXECUTABLE theta_driver_node)
ament_target_dependencies(theta_driver_lib
  ${dependencies}
  OpenCV
  Boost
)
target_link_libraries(theta_driver_lib
  ${GSTREAMER_1_0_LIBRARIES}
  ${GSTREAMER_1_0_APP_LIBRARIES}
  LibUVC::UVCShared
)

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

# Install lib
install(TARGETS
  theta_driver_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )
  
# Install exec
install(TARGETS
  theta_driver_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ 
  DESTINATION include/
)

ament_export_include_directories(
  include
)

    
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
