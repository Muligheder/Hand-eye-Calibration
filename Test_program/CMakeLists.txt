cmake_minimum_required(VERSION 3.5)

project(Test_program)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Set build type to release
SET(CMAKE_BUILD_TYPE Release)
MESSAGE("-- Build type: " ${CMAKE_BUILD_TYPE})

find_package(realsense2)

find_package(OpenCV REQUIRED )

find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)
include_directories(${GLFW_INCLUDE_DIRS})


find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(ur_rtde REQUIRED)

add_executable(${PROJECT_NAME} "main.cpp")
target_link_libraries(${PROJECT_NAME} PRIVATE realsense2::realsense2 ${OpenCV_LIBS} ${PCL_LIBRARIES} ${GLFW_LIBRARIES} PRIVATE ur_rtde::rtde -lpthread -lm)




