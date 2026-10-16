cmake_minimum_required(VERSION 3.8)
project(shelf_world)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds models DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/generate_world.py DESTINATION lib/${PROJECT_NAME})

ament_package()
