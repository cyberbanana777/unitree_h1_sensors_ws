cmake_minimum_required(VERSION 3.8)
project(realsense_d435i_configs)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_launch_params test/test_launch_params.py)
  ament_add_pytest_test(test_presets test/test_presets.py)
endif()

ament_package()
