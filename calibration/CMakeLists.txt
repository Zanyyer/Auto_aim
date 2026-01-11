cmake_minimum_required(VERSION 3.16)

add_executable(camera_calib  camera_calib.cpp)
add_executable(capture  capture.cpp)

target_link_libraries(camera_calib PRIVATE
    ${OpenCV_LIBS}
)
target_link_libraries(capture PRIVATE
    ${OpenCV_LIBS}
)