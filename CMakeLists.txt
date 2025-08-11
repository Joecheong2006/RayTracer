cmake_minimum_required(VERSION 3.10)
project(raytracer)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)  # disables compiler-specific extensions

# === Optional ===
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(${CMAKE_SOURCE_DIR}/lib/glad/include)
include_directories(${CMAKE_SOURCE_DIR}/lib/glm)
include_directories(${CMAKE_SOURCE_DIR}/lib)

file(GLOB SOURCES_LEVEL0 "src/*.cpp")
file(GLOB SOURCES_LEVEL1 "src/**/*.cpp")

add_executable(raytracer 
    ${SOURCES_LEVEL0}
    ${SOURCES_LEVEL1}
    lib/glad/src/glad.c
)

target_include_directories(raytracer PRIVATE ${CMAKE_SOURCE_DIR}/src)

if (WIN32)
    target_link_directories(raytracer PRIVATE ${CMAKE_SOURCE_DIR}/lib/glfw)
    target_link_libraries(raytracer PRIVATE glfw3 opengl32)
elseif (APPLE)
    find_package(glfw3 REQUIRED)
    find_package(OpenGL REQUIRED)
    target_link_libraries(raytracer PRIVATE glfw OpenGL::GL)
endif()

