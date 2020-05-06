cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_listner)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
add_compile_options(-w)
add_compile_options(-std=c++11)
add_compile_options(-fpermissive)
add_compile_options(-pthread)


#find_package(PhoXi REQUIRED CONFIG PATHS "$ENV{PHOXI_CONTROL_PATH}")
find_package(catkin REQUIRED
        COMPONENTS
 #       phoxi_camera
        roscpp
        roslib
        rospy
        rospy
        std_msgs
        sensor_msgs
        pcl_ros
        pcl_conversions
        tf
        tf_conversions
  #      phoxi_camera
        )

include_directories(include
        ${catkin_INCLUDE_DIRS}
        ${PHOXI_INCLUDE_DIRS}
        )

generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
)

add_executable(
        ${PROJECT_NAME}
        src/pcl_listener.cpp
)

add_dependencies(
        ${PROJECT_NAME}
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${PROJECT_NAME}_gencfgv
        ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        debug
        optimized ${catkin_LIBRARIES}
        ${PHOXI_LIBRARY}
        ${Boost_LIBRARIES}
        rt
)

#add_executable(fusion_test src/fusion_test.cpp)
#target_link_libraries(fusion_test ${catkin_LIBRARIES} ${CERES_LIBRARIES})
#add_dependencies(fusion_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(cloud_meshing src/cloud_meshing.cpp)
target_link_libraries(cloud_meshing ${catkin_LIBRARIES} ${CERES_LIBRARIES})
add_dependencies(cloud_meshing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
