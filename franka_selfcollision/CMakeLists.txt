cmake_minimum_required(VERSION 3.5)

# Suppress CMP0002 warning
option(CHECK_TIDY "Enable clang-tidy checks" OFF)

project(franka_selfcollision)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(urdf_parser_plugin REQUIRED)
find_package(urdf REQUIRED)
find_package(srdfdom REQUIRED)

find_package(Franka 0.13.2 REQUIRED)

add_compile_definitions(PINOCCHIO_WITH_HPP_FCL)

add_library(${PROJECT_NAME} SHARED
    src/self_collision_checker.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} 
    pinocchio::pinocchio
)

add_executable(self_collision_node 
    src/collision_checker_simple.cpp
)
target_link_libraries(self_collision_node pinocchio::pinocchio)
ament_target_dependencies(self_collision_node
    rclcpp
    franka_msgs
    sensor_msgs
    control_msgs
    std_srvs
    Franka
)

add_executable(self_collision_node_fast
    src/collision_checker_fast.cpp    
    src/collision_checker_impl.cpp 
)
target_include_directories(self_collision_node_fast PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(self_collision_node_fast pinocchio::pinocchio)
ament_target_dependencies(self_collision_node_fast
    rclcpp
    franka_msgs
    sensor_msgs
    control_msgs
    std_srvs
    Franka
)

add_executable(self_collision_node_bullet
    src/collision_checker_bullet.cpp
)

ament_target_dependencies(self_collision_node_bullet
    rclcpp
    moveit_core
    franka_msgs
    sensor_msgs
    std_srvs
    control_msgs
    urdf
    srdfdom
    Franka
)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(TARGETS
        self_collision_node
        self_collision_node_fast
        self_collision_node_bullet
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch config
        DESTINATION share/${PROJECT_NAME}/)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_clang_format REQUIRED)
    find_package(ament_cmake_copyright REQUIRED)
    find_package(ament_cmake_cppcheck REQUIRED)
    find_package(ament_cmake_flake8 REQUIRED)
    find_package(ament_cmake_lint_cmake REQUIRED)
    find_package(ament_cmake_pep257 REQUIRED)
    find_package(ament_cmake_xmllint REQUIRED)

    set(CPP_DIRECTORIES src include)
    ament_clang_format(CONFIG_FILE ../.clang-format ${CPP_DIRECTORIES})
    ament_copyright(launch ${CPP_DIRECTORIES} package.xml)
    ament_cppcheck(${CPP_DIRECTORIES})
    ament_lint_cmake(CMakeLists.txt)
    ament_flake8()
    ament_pep257()
    ament_xmllint()
endif()


ament_export_include_directories(
        include
)
ament_export_libraries(
        ${PROJECT_NAME}
)

ament_package()
