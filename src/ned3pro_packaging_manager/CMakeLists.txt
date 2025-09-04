cmake_minimum_required(VERSION 3.22)
project(ned3pro_packaging_manager)

find_package(ament_cmake REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

# Install Python scripts so they are runnable with ros2 run
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts")
  install(
    PROGRAMS
      scripts/packaging_node_naive.py
      scripts/packaging_node_moveit2.py
      scripts/publish_robot_description.py
    DESTINATION lib/${PROJECT_NAME}
  )
endif()
