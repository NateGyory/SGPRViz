
# Structure for a ROS CMakeLists.txt file

# 1) Required CMake Version (cmake_minimum_required)
# 2) Package Name (project())
# 3) Find other CMake/Catkin packages needed for build (find_package())
# 4) Enable Python module support (catkin_python_setup())
# 5) Message/Service/Action Generators (add_message_files(), add_service_files(), add_action_files())
# 6) Invoke message/service/action generation (generate_messages())
# 7) Specify package build info export (catkin_package())
# 8) Libraries/Executables to build (add_library()/add_executable()/target_link_libraries())
# 9) Tests to build (catkin_add_gtest())
# 10) Install rules (install())

cmake_minimum_required(VERSION 3.16)
project(save_ply)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)

find_package(PCL REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp std_msgs sensor_msgs
)
find_package(Boost REQUIRED COMPONENTS program_options)
#add_service_files(
#  FILES
#  Eigenvalues.srv
#  PointClouds.srv
#)

#generate_messages(
#  DEPENDENCIES std_msgs sensor_msgs
#)

catkin_package(
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp rospy
)

#catkin_install_python(PROGRAMS
#  scripts/evaluation_service.py
#  scripts/point_cloud_viz_service.py
#  scripts/histogram_viz_service.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

# Main node files and executable
#file(GLOB_RECURSE SRCS CONFIGURE_DEPENDS "src/*.cpp")

add_executable(save_ply "src/save_ply.cpp")

target_link_libraries(save_ply PRIVATE
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    nlohmann_json::nlohmann_json
)

target_include_directories(save_ply PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_executable(find_loops "src/find_loops.cpp")

target_link_libraries(find_loops PRIVATE
    ${catkin_LIBRARIES}
    nlohmann_json::nlohmann_json
    Boost::program_options
)

target_include_directories(find_loops PRIVATE
    ${catkin_INCLUDE_DIRS}
)

add_executable(find_loop_ranges "src/find_loop_ranges.cpp")

target_link_libraries(find_loop_ranges PRIVATE
    ${catkin_LIBRARIES}
    nlohmann_json::nlohmann_json
)

target_include_directories(find_loop_ranges PRIVATE
    ${catkin_INCLUDE_DIRS}
)
