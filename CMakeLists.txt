cmake_minimum_required(VERSION 2.8.3)
project(awv_mos_lio)

option(USE_EVALUATION_POINT_TYPE "Use evaluation point type" OFF)
if(USE_EVALUATION_POINT_TYPE)
    add_compile_definitions(USE_EVALUATION_POINT_TYPE)
endif()

# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  # pcl library
  pcl_conversions
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
  # rosbag
  rosbag 
  image_transport 
)

find_package(TBB REQUIRED COMPONENTS tbb)
message(STATUS "The value of TBB_FOUND is: ${TBB_FOUND}")
message(STATUS "The value of TBB_INCLUDE_DIR is: ${TBB_INCLUDE_DIR}")
message(STATUS "The value of TBB_LIBRARY_DIRS is: ${TBB_LIBRARY_DIRS}")
message(STATUS "The value of TBB_LIBRARIES is: ${TBB_LIBRARIES}")

find_package(OpenMP REQUIRED)
# find_package(PCL REQUIRED QUIET)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)



add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)

# add_service_files(
#   DIRECTORY srv
#   FILES
#   save_map.srv
# )

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL GTSAM

  CATKIN_DEPENDS
  cv_bridge 
  image_transport 
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  visualization_msgs
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
  ${TBB_INCLUDE_DIR}
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
  ${TBB_LIBRARY_DIRS}
)



###########
## Build ##
###########

# Range Image Projection
# Feature Association
add_executable(${PROJECT_NAME}_FeatureExtractionModule src/FeatureExtractionModule.cpp)
add_dependencies(${PROJECT_NAME}_FeatureExtractionModule ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_FeatureExtractionModule PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_FeatureExtractionModule ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} TBB::tbb)


# Mapping Optimization
add_executable(${PROJECT_NAME}_mapOptmization 
  src/mapOptmization.cpp
  src/AWV_MOS.cpp
  )
add_dependencies(${PROJECT_NAME}_mapOptmization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_mapOptmization PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_mapOptmization Boost::timer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam TBB::tbb)

# IMU Preintegration
add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp)
target_link_libraries(${PROJECT_NAME}_imuPreintegration Boost::timer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam TBB::tbb)

# MOS evaluation
add_executable(mos_evaluation src/mos_evaluation.cpp)
add_dependencies(mos_evaluation ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(mos_evaluation ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} TBB::tbb)

# AWV_MOS alone
add_executable(run_online_mos 
  src/run_online_mos.cpp
  src/AWV_MOS.cpp
  )
add_dependencies(run_online_mos ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(run_online_mos PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(run_online_mos Boost::timer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam TBB::tbb)
