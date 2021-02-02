cmake_minimum_required(VERSION 3.5)

project(Test_program)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_CXX_STANDARD 17)

# Set build type to release
SET(CMAKE_BUILD_TYPE Release)
MESSAGE("-- Build type: " ${CMAKE_BUILD_TYPE})

find_package(realsense2)

find_package(OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})




find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)
include_directories(${GLFW_INCLUDE_DIRS})


find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(ur_rtde REQUIRED)

add_executable(${PROJECT_NAME} src/main.cpp )
target_link_libraries(${PROJECT_NAME} PRIVATE realsense2::realsense2 ${OpenCV_LIBS} ${PCL_LIBRARIES} ${GLFW_LIBRARIES} PRIVATE ur_rtde::rtde -lpthread -lm)

add_executable(Camera_calibration src/camera_calibration.cpp )
target_link_libraries(Camera_calibration PRIVATE realsense2::realsense2 ${OpenCV_LIBS} ${PCL_LIBRARIES} ${GLFW_LIBRARIES} PRIVATE ur_rtde::rtde -lpthread -lm)

add_executable(Hand_eye src/hand_eye_calibration.cpp )
target_link_libraries(Hand_eye PRIVATE realsense2::realsense2 ${OpenCV_LIBS} ${PCL_LIBRARIES} ${GLFW_LIBRARIES} PRIVATE ur_rtde::rtde -lpthread -lm)

add_executable(Parsing src/parsing_tcp_data.cpp )
target_link_libraries(Parsing PRIVATE realsense2::realsense2 ${OpenCV_LIBS} ${PCL_LIBRARIES} ${GLFW_LIBRARIES} PRIVATE ur_rtde::rtde -lpthread -lm)

add_executable(RTDE src/ur_robot_rtde_test.cpp )
target_link_libraries(RTDE PRIVATE realsense2::realsense2 ${OpenCV_LIBS} ${PCL_LIBRARIES} ${GLFW_LIBRARIES} PRIVATE ur_rtde::rtde -lpthread -lm)

add_executable(HET src/hand_eye_test.cpp )
target_link_libraries(HET PRIVATE realsense2::realsense2 ${OpenCV_LIBS} ${PCL_LIBRARIES} ${GLFW_LIBRARIES} PRIVATE ur_rtde::rtde -lpthread -lm)

add_executable(ADV_CALI src/adv_camera_calibration.cpp )
target_link_libraries(ADV_CALI PRIVATE realsense2::realsense2 ${OpenCV_LIBS} ${PCL_LIBRARIES} ${GLFW_LIBRARIES} PRIVATE ur_rtde::rtde -lpthread -lm)

add_executable(PCL src/pcl_test.cpp )
target_link_libraries(PCL PRIVATE realsense2::realsense2 ${OpenCV_LIBS} ${PCL_LIBRARIES} ${GLFW_LIBRARIES} PRIVATE ur_rtde::rtde -lpthread -lm)




