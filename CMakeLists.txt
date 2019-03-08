cmake_minimum_required(VERSION 2.8)
project(indemind_module)

add_compile_options(-std=c++11)
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

#find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  cv_bridge
  tf
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES indemind_module
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs geometry_msgs tf
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
#Eigen
include_directories("/usr/include/eigen3")
#Sophus
find_package(Sophus REQUIRED)
include_directories( ${Sophus_INCLUDE_DIRS})
include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/ORB_SLAM/include
${PROJECT_SOURCE_DIR}/ORB_SLAM
#${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
ORB_SLAM/src/FileYaml.cc
ORB_SLAM/src/System.cc
ORB_SLAM/src/Tracking.cc
ORB_SLAM/src/LocalMapping.cc
ORB_SLAM/src/LoopClosing.cc
ORB_SLAM/src/ORBextractor.cc
ORB_SLAM/src/ORBmatcher.cc
ORB_SLAM/src/FrameDrawer.cc
ORB_SLAM/src/Converter.cc
ORB_SLAM/src/MapPoint.cc
ORB_SLAM/src/KeyFrame.cc
ORB_SLAM/src/Map.cc
ORB_SLAM/src/MapDrawer.cc
ORB_SLAM/src/Optimizer.cc
ORB_SLAM/src/PnPsolver.cc
ORB_SLAM/src/Frame.cc
ORB_SLAM/src/KeyFrameDatabase.cc
ORB_SLAM/src/Sim3Solver.cc
ORB_SLAM/src/Initializer.cc
ORB_SLAM/src/Viewer.cc
)


target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
#${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${Sophus_LIBRARIES}
${PROJECT_SOURCE_DIR}/ORB_SLAM/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/ORB_SLAM/Thirdparty/g2o/lib/libg2o.so
)

# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/ORB_SLAM/Examples/RGB-D)

add_executable(rgbd_tum
ORB_SLAM/Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum ${PROJECT_NAME}
	${Sophus_LIBRARIES})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/ORB_SLAM/Examples/Stereo)

add_executable(stereo_kitti
ORB_SLAM/Examples/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti ${PROJECT_NAME}
	${Sophus_LIBRARIES})

add_executable(stereo_euroc
ORB_SLAM/Examples/Stereo/stereo_euroc.cc)
add_dependencies(stereo_euroc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(stereo_euroc ${PROJECT_NAME}
	${Sophus_LIBRARIES}
	${catkin_LIBRARIES})

add_executable(module_driver src/camera_driver.cpp)
add_dependencies(module_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(module_driver
	${PROJECT_SOURCE_DIR}/../../lib/1604/libindem.so
	${catkin_LIBRARIES}
	${PROJECT_SOURCE_DIR}/../../lib/1604/libboost_filesystem.so.1.58.0
	${PROJECT_SOURCE_DIR}/../../lib/1604/libboost_system.so.1.58.0
	${PROJECT_SOURCE_DIR}/../../lib/1604/libg3logger.so.1.3.0-0
	${PROJECT_SOURCE_DIR}/../../lib/1604/libnanomsg.so.5
	${PROJECT_SOURCE_DIR}/../../lib/1604/libopencv_core.so.3.4
	${PROJECT_SOURCE_DIR}/../../lib/1604/libopencv_imgproc.so.3.4
	${PROJECT_SOURCE_DIR}/../../lib/1604/libopencv_videoio.so.3.4
	pthread
	stdc++fs
	dl
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/ORB_SLAM/Examples/Monocular)

add_executable(mono_tum
ORB_SLAM/Examples/Monocular/mono_tum.cc)
target_link_libraries(mono_tum 
${Sophus_LIBRARIES}
${PROJECT_NAME})

add_executable(mono_kitti
ORB_SLAM/Examples/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti 
${Sophus_LIBRARIES}
${PROJECT_NAME})

add_executable(mono_euroc
ORB_SLAM/Examples/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc 
${Sophus_LIBRARIES}
${PROJECT_NAME})

