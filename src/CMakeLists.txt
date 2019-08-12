add_executable(thesis_demo
	circular_movement_component.h
	circular_movement_component.cpp
    thesis_demo.cpp
)

target_link_libraries(thesis_demo
    pixel_engine
    ${OPENGL_LIBRRIES}
    gl3w
    glfw
    gflags
    glog
)