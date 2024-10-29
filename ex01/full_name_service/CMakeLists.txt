cmake_minimum_required(VERSION 3.5)
project(full_name_service)

# Установите необходимые зависимости
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)  # может понадобиться для других типов сообщений, если вы будете их использовать
find_package(full_name_message REQUIRED) 



# Добавляем исполняемые файлы для сервиса и клиента
add_executable(service_name src/service_name.cpp)
ament_target_dependencies(service_name rclcpp full_name_message)

add_executable(client_name src/client_name.cpp)
ament_target_dependencies(client_name rclcpp full_name_message)

install(TARGETS
  service_name
  client_name
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()