cmake_minimum_required(VERSION 3.5)
project(rpi_manager)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetSafety.srv"
)

install(
  PROGRAMS
    scripts/detection_naive.py
    scripts/detection.py
    scripts/detection_test.py
    scripts/hmi_standard.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()


