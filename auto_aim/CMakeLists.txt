cmake_minimum_required(VERSION 3.16)

# 模块名
project(auto_aim)

# 收集源文件
file(GLOB AUTO_AIM_SOURCES_FILES
    "src/*.cpp"
)

# 生成库
add_library(auto_aim
    ${AUTO_AIM_SOURCES_FILES}
)

# 头文件路径
target_include_directories(auto_aim
    PUBLIC
        include
)

# 依赖库
target_link_libraries(auto_aim
    PUBLIC
        opencv_core
        opencv_imgproc
        opencv_calib3d
        # ${OpenCV_LIBS}
        Eigen3::Eigen
        io
)