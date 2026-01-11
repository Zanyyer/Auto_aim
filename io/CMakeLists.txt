add_library(io
    usbcamera/usbcamera.cpp
    serials/serial.cpp
)
  
target_include_directories(io PUBLIC
    ${PROJECT_SOURCE_DIR}
)

target_link_libraries(io PUBLIC
    ${OpenCV_LIBS}
    # Eigen3::Eigen
    yaml-cpp
)
