cmake_minimum_required(VERSION 3.16)
project(trajectory_player LANGUAGES CXX)

find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_library(TrajectoryPlayer SHARED trajectory_player.cc)
target_compile_features(TrajectoryPlayer PRIVATE cxx_std_17)
target_link_libraries(TrajectoryPlayer PRIVATE
  gz-plugin2::register
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

install(TARGETS TrajectoryPlayer DESTINATION lib)
