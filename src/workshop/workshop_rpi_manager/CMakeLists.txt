cmake_minimum_required(VERSION 3.5)
project(workshop_rpi_manager)

find_package(ament_cmake REQUIRED)

ament_package()



if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts")
  install(
    PROGRAMS
      scripts/detection.py
      scripts/hmi_standard.py
    DESTINATION lib/${PROJECT_NAME}
  )
endif()
