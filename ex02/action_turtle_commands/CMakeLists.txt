cmake_minimum_required(VERSION 3.8)
project(action_turtle_commands)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(action_turtle_commands_dop REQUIRED)
find_package(turtlesim REQUIRED)


# Добавление исполняемого файла для клиента
add_executable(action_turtle_client src/action_turtle_client.cpp)
ament_target_dependencies(action_turtle_client geometry_msgs rclcpp rclcpp_action std_msgs action_turtle_commands_dop)

# Добавление исполняемого файла для сервера
add_executable(action_turtle_server src/action_turtle_server.cpp)
ament_target_dependencies(action_turtle_server turtlesim geometry_msgs rclcpp rclcpp_action std_msgs action_turtle_commands_dop)

install(TARGETS
  action_turtle_client
  action_turtle_server
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
