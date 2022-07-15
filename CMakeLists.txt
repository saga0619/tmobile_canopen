cmake_minimum_required(VERSION 3.0.2)
project(cobra4812_canopen)

find_package(Threads REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
    canopen_402
    canopen_master
)

catkin_package()

include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(cobra4812_canopen
  src/main.cpp
)
target_link_libraries(cobra4812_canopen
  ${catkin_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)
