cmake_minimum_required(VERSION 3.8)
project(coco_simulations)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
ament_package()
