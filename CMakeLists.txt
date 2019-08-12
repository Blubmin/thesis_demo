cmake_minimum_required(VERSION 3.14)
project(thesis_demo CXX)


# OpenGL
find_package(OpenGL REQUIRED)

# Boost
find_package(Boost REQUIRED filesystem)

# Third Party Libraries
add_definitions(-DImDrawIdx=uint32_t)
add_definitions(-DIMGUI_DEFINE_MATH_OPERATORS)
include_directories(pixel_engine)
include_directories(pixel_engine/third_party)
include_directories(pixel_engine/third_party/assimp/include)
include_directories(pixel_engine/third_party/eigen)
include_directories(pixel_engine/third_party/gl3w/include)
include_directories(pixel_engine/third_party/glog/src)
include_directories(pixel_engine/third_party/gflags/src)
add_subdirectory(pixel_engine)

add_subdirectory(src)