cmake_minimum_required(VERSION 3.16)

project(task7)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 第三方依赖（只在顶层找）
find_package(OpenCV   REQUIRED)
find_package(Eigen3   REQUIRED)
find_package(yaml-cpp REQUIRED)

# 子模块
add_subdirectory(io)
add_subdirectory(calibration)
add_subdirectory(auto_aim)
add_subdirectory(test)

# 主程序
add_executable(task7_main
    src/main.cpp
)

target_link_libraries(task7_main PRIVATE
    io
    auto_aim
)
