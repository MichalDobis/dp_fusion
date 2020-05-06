cmake_minimum_required(VERSION 2.8.3)
project(tf_logger)

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
        phoxi_camera
        pho_robot_loader
        )


set(PHO_ALPHAROOT $ENV{HOME})
set(ThirdPartyDIR "${PHO_ALPHAROOT}/3rdParty_REPO_gcc${CMAKE_CXX_COMPILER_VERSION}")
set(EIGEN_INCLUDE_DIR "${ThirdPartyDIR}/Eigen-3.2.9")
set(EIGEN_INCLUDE_DIRS "${ThirdPartyDIR}/Eigen-3.2.9")
set(GLOG_INCLUDE_DIR "/usr/include")

set(Ceres_DIR "${ThirdPartyDIR}/ceres-solver-1.12.0/lib/cmake/Ceres")
find_package(Ceres REQUIRED)



include_directories(
        ${catkin_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
)
catkin_package()

add_executable(tf_logger_sim src/tf_logger_sim.cpp)
target_link_libraries(tf_logger_sim  ${catkin_LIBRARIES})
add_dependencies(tf_logger_sim  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(tf_logger src/tf_logger)
target_link_libraries(tf_logger  ${catkin_LIBRARIES})
add_dependencies(tf_logger  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ax_bx_calibration_data_logger src/ax_bx_calibration_data_logger.cpp)
target_link_libraries(ax_bx_calibration_data_logger  ${catkin_LIBRARIES})
add_dependencies(ax_bx_calibration_data_logger  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${CERES_LIBRARIES})
