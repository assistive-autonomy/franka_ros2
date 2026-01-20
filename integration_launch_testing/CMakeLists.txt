cmake_minimum_required(VERSION 3.5)

# Suppress CMP0002 warning
option(CHECK_TIDY "Enable clang-tidy checks" OFF)
option(CMAKE_EXPORT_COMPILE_COMMANDS "Enable compile commands export" OFF)

project(integration_launch_testing)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
    install(
      DIRECTORY test
      DESTINATION share/${PROJECT_NAME}
    )

    # Install the test runner and generic test as executables
    install(PROGRAMS
      test/run_all_integration_tests.py
      DESTINATION lib/${PROJECT_NAME}
      RENAME run_all_integration_tests
    )

    install(PROGRAMS
      test/generic_controller_test.py
      DESTINATION lib/${PROJECT_NAME}
      RENAME test_controller
    )

    # Keep gripper test as separate executable (uses different test pattern)
    install(PROGRAMS
      test/test_franka_gripper_position.py
      DESTINATION lib/${PROJECT_NAME}
      RENAME test_gripper_position
    )

    find_package(ament_cmake_copyright REQUIRED)
    find_package(ament_cmake_flake8 REQUIRED)
    find_package(ament_cmake_lint_cmake REQUIRED)
    find_package(ament_cmake_pep257 REQUIRED)
    find_package(ament_cmake_xmllint REQUIRED)
    find_package(launch_testing_ament_cmake)

    ament_copyright(package.xml)
    ament_lint_cmake(CMakeLists.txt)
    ament_flake8()
    ament_pep257()
    ament_xmllint()
endif()

ament_export_libraries(
        ${PROJECT_NAME}
)
ament_package()