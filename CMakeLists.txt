cmake_minimum_required(VERSION 3.1...3.15)

project(GoldoBot VERSION 1.0 LANGUAGES C CXX)

# If you set any CMAKE_ variables, that can go here.

# Find packages go here.
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Inc)

add_subdirectory(Middlewares)
add_subdirectory(goldobot)