cmake_minimum_required(VERSION 3.8)
project(franka_moveit_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(graph_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(bondcpp REQUIRED)

set(executable_name franka_moveit_controller)
set(library_name ${executable_name}_core)

add_executable(${executable_name} src/main.cpp) #src/pick_place.cpp)

add_executable(fk_main src/fk_main.cpp)

target_include_directories(fk_main
PUBLIC include)

ament_target_dependencies(fk_main
ament_cmake
rclcpp
rclcpp_action
moveit_core
moveit_ros_planning
moveit_ros_planning_interface
control_msgs)

add_library(${library_name} SHARED
  src/moveit_controller.cpp
)

target_include_directories(${library_name}
PUBLIC include)

ament_target_dependencies(${library_name}
ament_cmake
rclcpp
rclcpp_lifecycle
rclcpp_action
moveit_core
moveit_ros_planning
moveit_ros_planning_interface
control_msgs
bondcpp)




target_link_libraries(${executable_name} ${library_name})
rclcpp_components_register_nodes(${library_name} "franka_example_controllers::MoveItContoller")


install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name} fk_main
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
         launch
         DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
