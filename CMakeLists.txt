cmake_minimum_required(VERSION 2.8.3)
project(theta_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  std_srvs
  sensor_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)
find_package(Boost 1.53 REQUIRED system thread timer)
find_package(libuvc REQUIRED)

find_package(PkgConfig)
pkg_check_modules(GSTREAMER_1_0 gstreamer-1.0)
pkg_check_modules(GSTREAMER_1_0_APP gstreamer-app-1.0)

include_directories(
  include
  3rd/libuvc-theta-sample/gst
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GSTREAMER_1_0_INCLUDE_DIRS}
)

catkin_package()

add_library(ThetaDriverNodelet src/theta_driver_nodelet.cpp)

target_link_libraries(ThetaDriverNodelet
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GSTREAMER_1_0_LIBRARIES}
  ${GSTREAMER_1_0_APP_LIBRARIES}
  LibUVC::UVCShared
  ${OpenCV_LIBS})

add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
  3rd/libuvc-theta-sample/gst/thetauvc.c)

target_link_libraries(${PROJECT_NAME}_node
  ThetaDriverNodelet)

install(TARGETS ${PROJECT_NAME}_node ThetaDriverNodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
