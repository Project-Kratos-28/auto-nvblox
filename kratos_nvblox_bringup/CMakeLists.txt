cmake_minimum_required(VERSION 3.8)
project(kratos_nvblox_bringup NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/throttle_relay.py DESTINATION lib/${PROJECT_NAME})

ament_package()
