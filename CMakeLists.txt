cmake_minimum_required(VERSION 3.16)
project(autonomy_planner CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra)

add_library(ap_core
  src/grid_map.cpp
  src/dubins.cpp
  src/hybrid_astar.cpp
  src/controller.cpp
  src/simulator.cpp
  src/scenarios.cpp)
target_include_directories(ap_core PUBLIC include)

add_executable(planner_demo src/main.cpp)
target_link_libraries(planner_demo ap_core)

enable_testing()

add_executable(test_dubins tests/test_dubins.cpp)
target_include_directories(test_dubins PRIVATE tests)
target_link_libraries(test_dubins ap_core)
add_test(NAME dubins COMMAND test_dubins)

add_executable(test_planner tests/test_planner.cpp)
target_include_directories(test_planner PRIVATE tests)
target_link_libraries(test_planner ap_core)
add_test(NAME planner COMMAND test_planner)
