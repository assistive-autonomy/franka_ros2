cmake_minimum_required(VERSION 3.5)
project(franka_mobile_example_controllers)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(CHECK_TIDY "Adds clang-tidy tests" OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(
        ${PROJECT_NAME}
        SHARED
        src/guiding_example_controller.cpp
        src/joint_velocity_example_controller.cpp
)

target_include_directories(
        ${PROJECT_NAME}
        PUBLIC
        include
        ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(
        ${PROJECT_NAME}
        controller_interface
        franka_msgs
        franka_semantic_components
        hardware_interface
        moveit_core
        moveit_msgs
        pinocchio
        pluginlib
        rclcpp
        rclcpp_lifecycle
        std_srvs
)

pluginlib_export_plugin_description_file(
        controller_interface franka_mobile_example_controllers.xml)

install(
        TARGETS
        ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

install(
        DIRECTORY include/
        DESTINATION include
)

# if(BUILD_TESTING)
#     find_package(ament_cmake_clang_format REQUIRED)
#     find_package(ament_cmake_copyright REQUIRED)
#     find_package(ament_cmake_cppcheck REQUIRED)
#     find_package(ament_cmake_flake8 REQUIRED)
#     find_package(ament_cmake_lint_cmake REQUIRED)
#     find_package(ament_cmake_pep257 REQUIRED)
#     find_package(ament_cmake_xmllint REQUIRED)
#     find_package(ament_cmake_gmock REQUIRED)
#     find_package(controller_manager REQUIRED)
#     find_package(ros2_control_test_assets REQUIRED)

#     set(CPP_DIRECTORIES src include)
#     ament_clang_format(CONFIG_FILE ../.clang-format ${CPP_DIRECTORIES})
#     ament_copyright(src ${CPP_DIRECTORIES} package.xml)
#     ament_cppcheck(${CPP_DIRECTORIES})
#     ament_lint_cmake(CMakeLists.txt)
#     ament_flake8()
#     ament_pep257()
#     ament_xmllint()
#     target_include_directories(${PROJECT_NAME}_test PRIVATE include)
#     ament_target_dependencies(${PROJECT_NAME}_test
#         controller_manager
#         ros2_control_test_assets
#     )

#     if(CHECK_TIDY)
#         find_package(ament_cmake_clang_tidy REQUIRED)
#         set(ament_cmake_clang_tidy_CONFIG_FILE ../.clang-tidy)
#         ament_clang_tidy(${CMAKE_BINARY_DIR} --jobs 6)
#     endif()
# endif()

ament_export_include_directories(
        include
)
ament_export_libraries(
        ${PROJECT_NAME}
)
ament_export_dependencies(
        controller_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        hardware_interface
        moveit_core
)

# Individual Nodes
add_executable(twist_example_controller src/twist_example_controller.cpp)
ament_target_dependencies(
    twist_example_controller
    geometry_msgs
    rclcpp
)

target_include_directories(
        twist_example_controller
        PUBLIC
        include
        ${EIGEN3_INCLUDE_DIRS}
)

install(
        TARGETS
        twist_example_controller
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
