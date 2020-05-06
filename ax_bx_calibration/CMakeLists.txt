cmake_minimum_required(VERSION 2.8.3)
project(ax_bx_calibration)


catkin_package(
        INCLUDE_DIRS ../ax_bx_calibration/include
)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()



find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        tf
        pho_robot_loader
        )
set(PHO_ALPHAROOT $ENV{HOME})
set(ThirdPartyDIR "${PHO_ALPHAROOT}/3rdParty_REPO_gcc${CMAKE_CXX_COMPILER_VERSION}")
set(EIGEN_INCLUDE_DIR "${ThirdPartyDIR}/Eigen-3.2.9")
set(EIGEN_INCLUDE_DIRS "${ThirdPartyDIR}/Eigen-3.2.9")
set(GLOG_INCLUDE_DIR "/usr/include")

set(Ceres_DIR "${ThirdPartyDIR}/ceres-solver-1.12.0/lib/cmake/Ceres")
find_package(Ceres REQUIRED)

install(DIRECTORY ../ax_bx_calibration/include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE
        )


include_directories(
        include/ax_bx_calibration
        ${catkin_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
)
#

add_executable(calibration_node src/calibration_node.cpp)
target_link_libraries(calibration_node ${catkin_LIBRARIES} ${CERES_LIBRARIES})
add_dependencies(calibration_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(camera_data_to_points src/camera_data_to_points)
target_link_libraries(camera_data_to_points  ${catkin_LIBRARIES})
add_dependencies(camera_data_to_points  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
