cmake_minimum_required(VERSION 3.5)
project(robucar_bringup)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_module(python/robucar_bringup.py)

install(PROGRAMS
  scripts/generate_configuration_file.py
  scripts/generate_ros2_control_description.py
  scripts/generate_urdf_description.py
  scripts/urdf_description.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_urdf_description test/test_urdf_description.py)
endif()

ament_package()
