cmake_minimum_required(VERSION 3.10)
project(raytracer)

set(CMAKE_CXX_STANDARD 11)

include_directories(${CMAKE_SOURCE_DIR}/lib/glad/include)
include_directories(${CMAKE_SOURCE_DIR}/lib)

add_executable(raytracer src/main.cpp lib/glad/src/glad.c)

if (WIN32)
    target_link_directories(raytracer PRIVATE ${CMAKE_SOURCE_DIR}/lib/glfw)
    target_link_libraries(raytracer PRIVATE glfw3 opengl32)
elseif (APPLE)
    find_package(glfw3 REQUIRED)
    find_package(OpenGL REQUIRED)
    target_link_libraries(raytracer PRIVATE glfw OpenGL::GL)
endif()

