cmake_minimum_required(VERSION 3.15)
project(realense)
file(GLOB SRC ./src/*.cpp)
add_executable(${PROJECT_NAME} ${SRC})
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(fmt REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC  realsense2
                                         ${OpenCV_LIBS}
                                          fmt::fmt )

