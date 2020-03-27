cmake_minimum_required(VERSION 2.8.3)
project(global_localization)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  std_msgs
  tf
)

find_package( OpenCV REQUIRED)
find_package( Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(visual_odometry src/main.cpp src/dataPrepare.cpp src/vo.cpp)
target_link_libraries(visual_odometry ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(depth_test src/depthImage.cpp)
target_link_libraries(depth_test ${catkin_LIBRARIES} ${OpenCV_LIBS})
