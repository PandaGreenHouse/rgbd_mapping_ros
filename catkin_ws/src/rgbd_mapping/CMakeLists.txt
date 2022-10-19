cmake_minimum_required(VERSION 2.8.3)
project(rgbd_mapping)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge roscpp rospy sensor_msgs std_msgs std_srvs nav_msgs geometry_msgs visualization_msgs
  image_transport tf tf_conversions tf2_ros eigen_conversions pcl_conversions
  pcl_ros nodelet genmsg image_geometry rviz
)
find_package(OpenCV REQUIRED)
#FIND_PACKAGE(PCL 1.7 REQUIRED QUIET COMPONENTS common io kdtree search surface filters registration sample_consensus segmentation )
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
pkg_check_modules(OGRE OGRE)
include_directories( ${OGRE_INCLUDE_DIRS} )
link_directories( ${OGRE_LIBRARY_DIRS} )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()
################################################
## Declare ROS messages, services and actions ##
################################################
find_package(message_generation)
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  TexturedQuad.msg
  TexturedQuadArray.msg
  Info.msg
  KeyPoint.msg
  MapGraph.msg
  Link.msg
  OdomInfo.msg
  Point2f.msg
  Point3f.msg
  RGBDImage.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
)
catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES rgbd_mapping
  CATKIN_DEPENDS cv_bridge roscpp rospy sensor_msgs std_msgs std_srvs nav_msgs geometry_msgs visualization_msgs
                 image_transport tf tf_conversions tf2_ros eigen_conversions pcl_conversions
                 pcl_ros image_geometry ${optional_dependencies}
  DEPENDS system_lib OpenCV
)
include_directories(
${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
# libraries
SET(Libraries
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
#   ${RTABMap_LIBRARIES}
)
#add_library(rgbd_mappingplugin src/plugin.cpp src/main_process.cpp src/util.cpp src/CBoundingBox.cpp)
#add_dependencies(rgbd_mappingplugin ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
#target_link_libraries(rgbd_mappingplugin ${Libraries})
add_executable(rgbd_mapping src/rgbd_mapping.cpp src/main_process.cpp src/util.cpp src/CBoundingBox.cpp src/CQuad.cpp src/csegment.cpp src/depth_edge.cpp src/planarSegment)
add_dependencies(rgbd_mapping ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(rgbd_mapping ${Libraries})



