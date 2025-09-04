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
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


