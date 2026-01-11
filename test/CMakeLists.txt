# ---------- test_camera ----------
add_executable(test_camera
    test_camera.cpp
)

target_link_libraries(test_camera
    PRIVATE
        io
)

# ---------- test_detector ----------
add_executable(test_detector
    test_detector.cpp
)

target_link_libraries(test_detector
    PRIVATE
        auto_aim
        io
        ${OpenCV_LIBS}
        Eigen3::Eigen
        yaml-cpp
)

# ---------- test_solver ----------
add_executable(test_solver
    test_solver.cpp
)

target_link_libraries(test_solver
    PRIVATE
        io
        auto_aim
        ${OpenCV_LIBS}
)
